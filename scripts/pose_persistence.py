#!/usr/bin/env python3
"""
Pose Persistence Node
---------------------
Continuously saves the robot's pose (via TF lookup: map -> base_link) to a
local SQLite database. On startup, publishes the last known pose to /initialpose
so that localization can recover after an unexpected shutdown.

Parameters:
    db_path              – path to the SQLite file  (default: ~/.warebot/pose_history.db)
    save_interval        – seconds between saves    (default: 2.0)
    map_frame            – TF parent frame           (default: map)
    robot_frame          – TF child frame            (default: base_link)
    publish_initial_pose – publish last pose on boot (default: True)
    max_history_rows     – DB row cap                (default: 50000)
    min_move_threshold   – metres; skip save if moved less (default: 0.001)
"""

import os
import signal
import sqlite3
import time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from std_srvs.srv import Trigger
from vizanti_server.srv import GetLastPose


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


class PosePersistence(Node):

    # ------------------------------------------------------------------ init
    def __init__(self):
        super().__init__("pose_persistence")

        # ---------- parameters ----------
        self.declare_parameter("db_path", "~/.warebot/pose_history.db")
        self.declare_parameter("save_interval", 2.0)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("robot_frame", "base_footprint")
        self.declare_parameter("publish_initial_pose", True)
        self.declare_parameter("max_history_rows", 50000)
        self.declare_parameter("min_move_threshold", 0.001)

        self.db_path = os.path.expanduser(
            self.get_parameter("db_path").value
        )
        self.save_interval = self.get_parameter("save_interval").value
        self.map_frame = self.get_parameter("map_frame").value
        self.robot_frame = self.get_parameter("robot_frame").value
        self.publish_on_start = self.get_parameter("publish_initial_pose").value
        self.max_history = self.get_parameter("max_history_rows").value
        self.min_move = self.get_parameter("min_move_threshold").value

        # ---------- database ----------
        os.makedirs(os.path.dirname(self.db_path), exist_ok=True)
        self._init_db()

        # ---------- TF ----------
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---------- services ----------
        cb_group = ReentrantCallbackGroup()
        self.create_service(
            GetLastPose,
            "vizanti/get_last_pose",
            self._srv_get_last_pose,
            callback_group=cb_group,
        )
        self.create_service(
            Trigger,
            "vizanti/clear_pose_history",
            self._srv_clear_history,
            callback_group=cb_group,
        )

        # ---------- publisher for initial pose ----------
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )

        # ---------- timers ----------
        self.save_timer = self.create_timer(self.save_interval, self._tick_save)
        self.cleanup_timer = self.create_timer(3600.0, self._tick_cleanup)

        # ---------- internal state ----------
        self._last_x = None
        self._last_y = None
        self._tf_ok = False  # track if we ever got a valid TF

        self.get_logger().info(
            f"Pose persistence active  |  db={self.db_path}  "
            f"interval={self.save_interval}s  "
            f"tf={self.map_frame}->{self.robot_frame}"
        )

        # delayed initial-pose publish (wait for TF tree to populate)
        if self.publish_on_start:
            self._startup_timer = self.create_timer(
                5.0, self._publish_startup_pose
            )

    # ---------------------------------------------------------- database init
    def _init_db(self):
        conn = sqlite3.connect(self.db_path)
        conn.executescript(DB_SCHEMA)
        conn.close()

    def _connect(self):
        return sqlite3.connect(self.db_path)

    # --------------------------------------------------- periodic pose saving
    def _tick_save(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.map_frame, self.robot_frame, rclpy.time.Time()
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            if not self._tf_ok:
                self.get_logger().info(
                    f"Waiting for TF {self.map_frame}->{self.robot_frame} ...",
                    throttle_duration_sec=15.0,
                )
            return
        except Exception as exc:
            self.get_logger().error(
                f"TF error: {exc}", throttle_duration_sec=10.0
            )
            return

        self._tf_ok = True

        x = t.transform.translation.x
        y = t.transform.translation.y
        z = t.transform.translation.z
        qx = t.transform.rotation.x
        qy = t.transform.rotation.y
        qz = t.transform.rotation.z
        qw = t.transform.rotation.w

        # skip if the robot barely moved
        if self._last_x is not None:
            dx = abs(x - self._last_x)
            dy = abs(y - self._last_y)
            if dx < self.min_move and dy < self.min_move:
                return

        ts = time.time()
        try:
            conn = self._connect()
            conn.execute(
                "INSERT INTO pose_history "
                "(timestamp, frame_id, x, y, z, qx, qy, qz, qw) "
                "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)",
                (ts, self.map_frame, x, y, z, qx, qy, qz, qw),
            )
            conn.commit()
            conn.close()
        except Exception as exc:
            self.get_logger().error(
                f"DB write error: {exc}", throttle_duration_sec=10.0
            )
            return

        self._last_x = x
        self._last_y = y

    # --------------------------------------------------- periodic DB cleanup
    def _tick_cleanup(self):
        try:
            conn = self._connect()
            cur = conn.execute(
                "DELETE FROM pose_history WHERE id NOT IN "
                "(SELECT id FROM pose_history ORDER BY timestamp DESC LIMIT ?)",
                (self.max_history,),
            )
            deleted = cur.rowcount
            conn.commit()
            conn.close()
            if deleted > 0:
                self.get_logger().info(f"Pruned {deleted} old pose entries")
        except Exception as exc:
            self.get_logger().error(f"Cleanup error: {exc}")

    # --------------------------------------------- startup initial pose pub
    def _publish_startup_pose(self):
        self._startup_timer.cancel()  # one-shot

        row = self._fetch_last_row()
        if row is None:
            self.get_logger().info("No saved poses – skipping initial pose publish")
            return

        ts, frame, x, y, z, qx, qy, qz, qw = row

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = z
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        # conservative covariance
        msg.pose.covariance[0] = 0.25  # σ²_x
        msg.pose.covariance[7] = 0.25  # σ²_y
        msg.pose.covariance[35] = 0.07  # σ²_yaw

        self.initial_pose_pub.publish(msg)
        self.get_logger().info(
            f"Published last known pose as /initialpose  "
            f"x={x:.3f}  y={y:.3f}  (saved {time.ctime(ts)})"
        )

    # -------------------------------------------------------- ROS2 services
    def _fetch_last_row(self):
        """Return (timestamp, frame_id, x, y, z, qx, qy, qz, qw) or None."""
        try:
            conn = self._connect()
            cur = conn.execute(
                "SELECT timestamp, frame_id, x, y, z, qx, qy, qz, qw "
                "FROM pose_history ORDER BY timestamp DESC LIMIT 1"
            )
            row = cur.fetchone()
            conn.close()
            return row
        except Exception:
            return None

    def _srv_get_last_pose(self, _request, response):
        row = self._fetch_last_row()
        if row is None:
            response.success = False
            response.message = "No poses saved yet"
            return response

        ts, frame, x, y, z, qx, qy, qz, qw = row
        response.pose.header.frame_id = frame
        response.pose.header.stamp.sec = int(ts)
        response.pose.header.stamp.nanosec = int((ts % 1) * 1e9)
        response.pose.pose.position.x = x
        response.pose.pose.position.y = y
        response.pose.pose.position.z = z
        response.pose.pose.orientation.x = qx
        response.pose.pose.orientation.y = qy
        response.pose.pose.orientation.z = qz
        response.pose.pose.orientation.w = qw
        response.success = True
        response.message = f"Last pose from {time.ctime(ts)}"
        return response

    def _srv_clear_history(self, _request, response):
        try:
            conn = self._connect()
            conn.execute("DELETE FROM pose_history")
            conn.commit()
            conn.close()
            self._last_x = None
            self._last_y = None
            response.success = True
            response.message = "Pose history cleared"
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        return response

    # ------------------------------------------------ graceful final save
    def save_final_pose(self):
        """Called right before shutdown to persist the very last reading."""
        self._tick_save()
        self.get_logger().info("Final pose saved to database")


# ================================================================= main
def main(args=None):
    rclpy.init(args=args)
    node = PosePersistence()

    def _shutdown_handler(signum, frame):
        node.save_final_pose()
        raise SystemExit

    signal.signal(signal.SIGTERM, _shutdown_handler)

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.save_final_pose()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
