#!/usr/bin/env python3
"""
VPS Flask Server  –  standalone, no ROS2 dependency
====================================================
Serves the Vizanti web UI and REST APIs.  The browser connects to this server
for HTTP assets and to the Pi's rosbridge (proxied via Nginx) for live
ROS2 topic data over WebSocket.

Usage:
    python3 server.py                        # uses ./config.json
    python3 server.py --config /etc/warebot/config.json
"""

import os
import sys
import json
import time
import signal
import sqlite3
import logging
import argparse
import threading

from pathlib import Path
from flask import Flask, render_template, send_from_directory, make_response, request, jsonify
from waitress.server import create_server

# ── Defaults ──────────────────────────────────────────────────────────────────
DEFAULT_CONFIG = {
    "host": "0.0.0.0",
    "port": 6700,
    "port_rosbridge": 6701,
    "base_url": "",
    "compression": "none",
    "flask_debug": False,
    "default_widget_config": "",
    "pi_bridge_url": "http://localhost:6780",
    "pose_db_path": "~/.warebot/pose_history.db",
}

# ── Config Loading ────────────────────────────────────────────────────────────
def load_config(config_path: str) -> dict:
    """Load config from JSON, falling back to defaults for missing keys."""
    cfg = dict(DEFAULT_CONFIG)
    if config_path and os.path.exists(config_path):
        with open(config_path) as f:
            user = json.load(f)
        # strip comment keys
        user = {k: v for k, v in user.items() if not k.startswith("_")}
        cfg.update(user)
        print(f"[config] Loaded {config_path}")
    else:
        print(f"[config] No config file at {config_path!r}, using defaults")

    # env overrides (useful in Docker / systemd)
    for key in cfg:
        env = f"WAREBOT_{key.upper()}"
        if env in os.environ:
            val = os.environ[env]
            # coerce types
            if isinstance(cfg[key], bool):
                val = val.lower() in ("1", "true", "yes")
            elif isinstance(cfg[key], int):
                val = int(val)
            cfg[key] = val
            print(f"[config] ENV override {env}={val}")

    return cfg


# ── Public dir discovery ──────────────────────────────────────────────────────
def get_public_dir():
    """Locate the public/ folder next to this script or in the repo root."""
    p = Path(__file__).resolve()
    # vps/server.py  →  repo_root/public
    repo_public = p.parents[1] / "public"
    if repo_public.exists():
        return str(repo_public)
    # fallback: same directory
    local = p.parent / "public"
    if local.exists():
        return str(local)
    raise FileNotFoundError(
        f"Cannot find public/ directory. Looked in:\n  {repo_public}\n  {local}"
    )


# ── Flask application ────────────────────────────────────────────────────────
def create_app(cfg: dict) -> Flask:
    public = get_public_dir()
    app = Flask(__name__, static_folder=public, template_folder=public)
    app.debug = cfg["flask_debug"]

    base = cfg["base_url"]
    default_cfg_path = cfg["default_widget_config"]
    if default_cfg_path:
        default_cfg_path = os.path.expanduser(default_cfg_path)
    else:
        default_cfg_path = os.path.join(public, "assets/default_layout.json")

    db_path = os.path.expanduser(cfg["pose_db_path"])
    pi_bridge = cfg["pi_bridge_url"].rstrip("/")

    # ── page routes ───────────────────────────────────────────────────────

    @app.route(base + "/")
    def index():
        return render_template("index.html", base_url=base)

    @app.route(base + "/templates/files")
    def list_template_files():
        return _get_files("templates", [".html", ".js", ".css"])

    @app.route(base + "/assets/robot_model/paths")
    def list_robot_model_files():
        templates_dir = os.path.join(public, "assets/robot_model")
        categorized_files = {"ground": [], "air": [], "sea": [], "misc": []}
        for root, dirs, files in os.walk(templates_dir):
            for f in files:
                if os.path.splitext(f)[1] == ".png":
                    rel = os.path.relpath(root, templates_dir)
                    cat = rel if rel in categorized_files else "misc"
                    if cat == ".":
                        cat = "misc"
                    categorized_files[cat].append(f)
        return _js_module(f"const categorizedPaths = {json.dumps(categorized_files)};\nexport default categorizedPaths;")

    @app.route(base + "/ros_launch_params")
    def ros_launch_params():
        params = {
            "port": cfg["port"],
            "port_rosbridge": cfg["port_rosbridge"],
            "compression": cfg["compression"],
        }
        return _js_module(f"const params = {json.dumps(params)};\nexport default params;")

    @app.route(base + "/default_widget_config")
    def get_default_widget_config():
        with open(default_cfg_path) as f:
            content = f.read()
        return _js_module(f"const content = {json.dumps(content)};\nexport default content;")

    # ── Pose REST API ─────────────────────────────────────────────────────

    @app.route(base + "/api/pose/last")
    def api_get_last_pose():
        if not os.path.exists(db_path):
            return jsonify(success=False, message="No pose database found"), 404
        try:
            conn = sqlite3.connect(db_path)
            row = conn.execute(
                "SELECT timestamp, frame_id, x, y, z, qx, qy, qz, qw "
                "FROM pose_history ORDER BY timestamp DESC LIMIT 1"
            ).fetchone()
            conn.close()
            if row is None:
                return jsonify(success=False, message="No poses saved yet"), 404
            return jsonify(
                success=True,
                pose=dict(
                    timestamp=row[0], frame_id=row[1],
                    x=row[2], y=row[3], z=row[4],
                    qx=row[5], qy=row[6], qz=row[7], qw=row[8],
                ),
            )
        except Exception as e:
            return jsonify(success=False, message=str(e)), 500

    @app.route(base + "/api/pose/history")
    def api_get_pose_history():
        if not os.path.exists(db_path):
            return jsonify(success=False, message="No pose database found"), 404
        limit = request.args.get("limit", 100, type=int)
        try:
            conn = sqlite3.connect(db_path)
            rows = conn.execute(
                "SELECT timestamp, frame_id, x, y, z, qx, qy, qz, qw "
                "FROM pose_history ORDER BY timestamp DESC LIMIT ?",
                (limit,),
            ).fetchall()
            conn.close()
            poses = [
                dict(
                    timestamp=r[0], frame_id=r[1],
                    x=r[2], y=r[3], z=r[4],
                    qx=r[5], qy=r[6], qz=r[7], qw=r[8],
                )
                for r in rows
            ]
            return jsonify(success=True, poses=poses, count=len(poses))
        except Exception as e:
            return jsonify(success=False, message=str(e)), 500

    @app.route(base + "/api/pose/receive", methods=["POST"])
    def api_receive_pose():
        """Endpoint called by Pi's pose_sync to push new pose records."""
        data = request.get_json(silent=True)
        if not data or "poses" not in data:
            return jsonify(success=False, message="Missing 'poses' array"), 400
        os.makedirs(os.path.dirname(db_path), exist_ok=True)
        try:
            conn = sqlite3.connect(db_path)
            conn.executescript(
                """CREATE TABLE IF NOT EXISTS pose_history (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    timestamp REAL NOT NULL,
                    frame_id TEXT NOT NULL,
                    x REAL NOT NULL, y REAL NOT NULL, z REAL NOT NULL,
                    qx REAL NOT NULL, qy REAL NOT NULL, qz REAL NOT NULL, qw REAL NOT NULL
                );
                CREATE INDEX IF NOT EXISTS idx_pose_ts ON pose_history(timestamp DESC);"""
            )
            for p in data["poses"]:
                conn.execute(
                    "INSERT INTO pose_history (timestamp,frame_id,x,y,z,qx,qy,qz,qw) "
                    "VALUES (?,?,?,?,?,?,?,?,?)",
                    (p["timestamp"], p["frame_id"],
                     p["x"], p["y"], p["z"],
                     p["qx"], p["qy"], p["qz"], p["qw"]),
                )
            # cleanup old rows
            conn.execute(
                "DELETE FROM pose_history WHERE id NOT IN "
                "(SELECT id FROM pose_history ORDER BY timestamp DESC LIMIT 50000)"
            )
            conn.commit()
            conn.close()
            return jsonify(success=True, received=len(data["poses"]))
        except Exception as e:
            return jsonify(success=False, message=str(e)), 500

    # ── static fallthrough ────────────────────────────────────────────────

    @app.route(base + "/<path:path>")
    def serve_static(path):
        return send_from_directory(public, path)

    # ── helpers ───────────────────────────────────────────────────────────

    def _js_module(body: str):
        resp = make_response(body)
        resp.headers["Content-Type"] = "application/javascript"
        return resp

    def _get_files(subdir, extensions):
        templates_dir = os.path.join(public, subdir)
        file_list = []
        for root, dirs, files in os.walk(templates_dir):
            for f in files:
                if os.path.splitext(f)[1] in extensions:
                    fp = os.path.join(root, f)
                    with open(fp) as fh:
                        file_list.append({
                            "path": os.path.relpath(fp, templates_dir),
                            "content": fh.read(),
                        })
        return _js_module(f"const files = {json.dumps(file_list)};\nexport default files;")

    return app


# ── Waitress server thread ───────────────────────────────────────────────────
class ServerThread(threading.Thread):
    def __init__(self, app, host="0.0.0.0", port=6700):
        super().__init__(daemon=True)
        self.app = app
        self.host = host
        self.port = port
        self._server = None

        self.log = logging.getLogger("waitress")
        self.log.setLevel(logging.INFO)
        handler = logging.StreamHandler()
        handler.setFormatter(logging.Formatter(
            "[%(levelname)s] [%(asctime)s] [waitress]: %(message)s"
        ))
        self.log.addHandler(handler)

    def run(self):
        self._server = create_server(self.app, host=self.host, port=self.port)
        self._server.run()

    def shutdown(self):
        if self._server:
            self._server.close()


# ── Entry point ──────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="WareBot VPS Web Server")
    parser.add_argument(
        "--config", "-c",
        default=os.path.join(os.path.dirname(__file__), "config.json"),
        help="Path to config.json (default: ./config.json)",
    )
    args = parser.parse_args()

    cfg = load_config(args.config)
    app = create_app(cfg)

    server = ServerThread(app, cfg["host"], cfg["port"])

    def sig_handler(sig, frame):
        print("\n[server] Shutting down...")
        server.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    print(f"[server] Flask serving on http://{cfg['host']}:{cfg['port']}{cfg['base_url']}")
    print(f"[server] Public dir: {get_public_dir()}")
    print(f"[server] rosbridge port advertised to browser: {cfg['port_rosbridge']}")
    print(f"[server] Pi bridge URL for pose sync: {cfg['pi_bridge_url']}")

    server.start()

    # Keep main thread alive for signal handling
    try:
        while True:
            time.sleep(1)
    except (KeyboardInterrupt, SystemExit):
        server.shutdown()


if __name__ == "__main__":
    main()
