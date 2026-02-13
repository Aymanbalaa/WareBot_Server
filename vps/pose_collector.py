#!/usr/bin/env python3
"""
VPS Pose Collector
-------------------
Connects to rosbridge (via the VPS nginx proxy at localhost:6701), subscribes
to '/vizanti/tf_consolidated' and saves the robot transform (map->robot_frame)
into an SQLite database on the VPS. This offloads DB work from the Pi.

Usage:
    python3 pose_collector.py               # use ./config.json defaults
    python3 pose_collector.py --host localhost --port 6701 --db ~/.warebot/pose_history.db

"""
import argparse
import json
import os
import sqlite3
import sys
import time

import roslibpy

DB_SCHEMA = """
CREATE TABLE IF NOT EXISTS pose_history (
    id        INTEGER PRIMARY KEY AUTOINCREMENT,
    timestamp REAL    NOT NULL,
    frame_id  TEXT    NOT NULL,
    x         REAL    NOT NULL,
    y         REAL    NOT NULL,
    z         REAL    NOT NULL,
    qx        REAL    NOT NULL,
    qy        REAL    NOT NULL,
    qz        REAL    NOT NULL,
    qw        REAL    NOT NULL
);
CREATE INDEX IF NOT EXISTS idx_pose_ts ON pose_history(timestamp DESC);
"""


class PoseCollector:
    def __init__(self, ros_host, ros_port, db_path, save_interval=2.0, map_frame='map', robot_frame='base_link', min_move=0.001, max_history=50000):
        self.db_path = os.path.expanduser(db_path)
        os.makedirs(os.path.dirname(self.db_path), exist_ok=True)
        self._init_db()

        self.ros = roslibpy.Ros(host=ros_host, port=int(ros_port))
        self.topic = None

        self.save_interval = float(save_interval)
        self.map_frame = map_frame
        self.robot_frame = robot_frame
        self.min_move = float(min_move)
        self.max_history = int(max_history)

        self._last_x = None
        self._last_y = None
        self._last_saved = 0.0

    def _init_db(self):
        conn = sqlite3.connect(self.db_path)
        conn.executescript(DB_SCHEMA)
        conn.close()

    def _connect(self):
        return sqlite3.connect(self.db_path)

    def start(self):
        print(f"[pose_collector] Connecting to rosbridge at {self.ros.host}:{self.ros.port}")
        self.ros.run()
        # subscribe
        self.topic = roslibpy.Topic(self.ros, '/vizanti/tf_consolidated', 'tf2_msgs/msg/TFMessage')
        self.topic.subscribe(self._on_tf)

        try:
            while True:
                time.sleep(1.0)
        except KeyboardInterrupt:
            print("[pose_collector] Shutting down...")
        finally:
            if self.topic:
                try:
                    self.topic.unsubscribe()
                except Exception:
                    pass
            try:
                self.ros.terminate()
            except Exception:
                pass

    def _on_tf(self, msg):
        # msg.transforms is a list of transforms
        now = time.time()
        # rate-limit saves to save_interval
        if now - self._last_saved < self.save_interval:
            return

        transforms = msg.get('transforms', [])
        target = None
        for t in transforms:
            child = t.get('child_frame_id') or ''
            if child.endswith(self.robot_frame) or child == self.robot_frame:
                target = t
                break

        if target is None:
            # try to find transform by header frame matching map and child containing robot_frame
            for t in transforms:
                hdr = t.get('header', {})
                if hdr.get('frame_id') == self.map_frame and (t.get('child_frame_id') == self.robot_frame):
                    target = t
                    break

        if target is None:
            return

        trans = target.get('transform', {})
        translation = trans.get('translation', {})
        rotation = trans.get('rotation', {})

        x = float(translation.get('x', 0.0))
        y = float(translation.get('y', 0.0))
        z = float(translation.get('z', 0.0))
        qx = float(rotation.get('x', 0.0))
        qy = float(rotation.get('y', 0.0))
        qz = float(rotation.get('z', 0.0))
        qw = float(rotation.get('w', 1.0))

        # skip if robot barely moved
        if self._last_x is not None:
            dx = abs(x - self._last_x)
            dy = abs(y - self._last_y)
            if dx < self.min_move and dy < self.min_move:
                return

        try:
            conn = self._connect()
            conn.execute(
                "INSERT INTO pose_history (timestamp, frame_id, x, y, z, qx, qy, qz, qw) VALUES (?,?,?,?,?,?,?,?,?)",
                (now, self.map_frame, x, y, z, qx, qy, qz, qw),
            )
            conn.commit()
            conn.execute(
                "DELETE FROM pose_history WHERE id NOT IN (SELECT id FROM pose_history ORDER BY timestamp DESC LIMIT ?)",
                (self.max_history,)
            )
            conn.commit()
            conn.close()
            self._last_saved = now
            self._last_x = x
            self._last_y = y
            print(f"[pose_collector] Saved pose x={x:.3f} y={y:.3f} ts={now}")
        except Exception as exc:
            print(f"[pose_collector] DB error: {exc}")


def main():
    parser = argparse.ArgumentParser(description='VPS Pose Collector')
    parser.add_argument('--host', default='localhost', help='rosbridge host (default localhost)')
    parser.add_argument('--port', default=6701, help='rosbridge port (default 6701)')
    parser.add_argument('--db', default='~/.warebot/pose_history.db', help='SQLite DB path')
    parser.add_argument('--interval', type=float, default=2.0, help='save interval seconds')
    parser.add_argument('--map-frame', default='map', help='map frame id')
    parser.add_argument('--robot-frame', default='base_link', help='robot frame id to find in TF')
    parser.add_argument('--min-move', type=float, default=0.001, help='min move threshold to save')
    args = parser.parse_args()

    collector = PoseCollector(
        ros_host=args.host,
        ros_port=args.port,
        db_path=args.db,
        save_interval=args.interval,
        map_frame=args.map_frame,
        robot_frame=args.robot_frame,
        min_move=args.min_move,
    )
    collector.start()


if __name__ == '__main__':
    main()
