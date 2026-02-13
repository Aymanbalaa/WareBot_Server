#!/usr/bin/env python3
"""
Pose Sync Client  –  runs on the Pi alongside pose_persistence
================================================================
Watches the local SQLite pose database and pushes new rows to the
VPS server's /api/pose/receive endpoint over HTTP.

This runs as a lightweight standalone script (no ROS2 dependency) so it
can be started via systemd or cron alongside the ROS2 launch.

Usage:
    python3 pose_sync.py                           # uses defaults
    python3 pose_sync.py --vps-url http://VPS:6700 --db ~/.warebot/pose_history.db
"""

import os
import sys
import time
import json
import signal
import sqlite3
import argparse
import urllib.request
import urllib.error

DEFAULT_VPS_URL = "http://localhost:6700"
DEFAULT_DB_PATH = "~/.warebot/pose_history.db"
DEFAULT_SYNC_INTERVAL = 5.0   # seconds between sync pushes
DEFAULT_BATCH_SIZE = 50       # max rows per HTTP POST


def sync_loop(vps_url: str, db_path: str, interval: float, batch_size: int):
    endpoint = vps_url.rstrip("/") + "/api/pose/receive"
    last_synced_id = 0

    # Try to recover last synced id from a marker file
    marker = db_path + ".sync_marker"
    if os.path.exists(marker):
        try:
            with open(marker) as f:
                last_synced_id = int(f.read().strip())
        except Exception:
            last_synced_id = 0

    print(f"[pose_sync] Syncing {db_path} → {endpoint}")
    print(f"[pose_sync] Starting from id > {last_synced_id}, interval={interval}s")

    running = True
    def _stop(sig, frame):
        nonlocal running
        running = False
    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    while running:
        if not os.path.exists(db_path):
            time.sleep(interval)
            continue

        try:
            conn = sqlite3.connect(db_path)
            rows = conn.execute(
                "SELECT id, timestamp, frame_id, x, y, z, qx, qy, qz, qw "
                "FROM pose_history WHERE id > ? ORDER BY id ASC LIMIT ?",
                (last_synced_id, batch_size),
            ).fetchall()
            conn.close()
        except Exception as e:
            print(f"[pose_sync] DB read error: {e}")
            time.sleep(interval)
            continue

        if not rows:
            time.sleep(interval)
            continue

        poses = []
        max_id = last_synced_id
        for r in rows:
            poses.append({
                "timestamp": r[1], "frame_id": r[2],
                "x": r[3], "y": r[4], "z": r[5],
                "qx": r[6], "qy": r[7], "qz": r[8], "qw": r[9],
            })
            max_id = max(max_id, r[0])

        payload = json.dumps({"poses": poses}).encode()
        req = urllib.request.Request(
            endpoint,
            data=payload,
            headers={"Content-Type": "application/json"},
            method="POST",
        )

        try:
            with urllib.request.urlopen(req, timeout=10) as resp:
                body = json.loads(resp.read())
                if body.get("success"):
                    last_synced_id = max_id
                    # persist marker
                    with open(marker, "w") as f:
                        f.write(str(last_synced_id))
                else:
                    print(f"[pose_sync] VPS rejected: {body.get('message')}")
        except urllib.error.URLError as e:
            print(f"[pose_sync] VPS unreachable: {e}")
        except Exception as e:
            print(f"[pose_sync] Sync error: {e}")

        time.sleep(interval)

    print("[pose_sync] Stopped.")


def main():
    parser = argparse.ArgumentParser(description="Pi → VPS Pose Sync")
    parser.add_argument("--vps-url", default=DEFAULT_VPS_URL,
                        help=f"VPS server URL (default: {DEFAULT_VPS_URL})")
    parser.add_argument("--db", default=DEFAULT_DB_PATH,
                        help=f"SQLite DB path (default: {DEFAULT_DB_PATH})")
    parser.add_argument("--interval", type=float, default=DEFAULT_SYNC_INTERVAL,
                        help=f"Sync interval seconds (default: {DEFAULT_SYNC_INTERVAL})")
    parser.add_argument("--batch", type=int, default=DEFAULT_BATCH_SIZE,
                        help=f"Max rows per push (default: {DEFAULT_BATCH_SIZE})")
    args = parser.parse_args()

    db_path = os.path.expanduser(args.db)
    sync_loop(args.vps_url, db_path, args.interval, args.batch)


if __name__ == "__main__":
    main()
