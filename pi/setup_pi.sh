#!/usr/bin/env bash
# ──────────────────────────────────────────────────────────────────────
# WareBot Pi Bridge Setup  –  Raspberry Pi (Ubuntu / Raspberry Pi OS)
# ──────────────────────────────────────────────────────────────────────
# Run as root or with sudo.
#
# What this does:
#   1. Sets up WireGuard to connect to VPS
#   2. Creates a systemd unit for pose_sync.py
#   3. Provides instructions for the ROS2 launch
#
# Prerequisites:
#   - ROS2 is already installed on the Pi
#   - vizanti_server package is built (colcon build)
#   - VPS public key is available
#
# Usage:
#   chmod +x setup_pi.sh
#   sudo ./setup_pi.sh
# ──────────────────────────────────────────────────────────────────────

set -euo pipefail

WG_INTERFACE="wg0"
WAREBOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"

echo "════════════════════════════════════════════════════════"
echo "  WareBot Pi Bridge Setup"
echo "════════════════════════════════════════════════════════"

# ── 1. Install WireGuard ─────────────────────────────────────────────
echo "[1/3] Installing WireGuard..."
apt-get update -qq
apt-get install -y -qq wireguard > /dev/null

# ── 2. WireGuard config ─────────────────────────────────────────────
if [ ! -f "/etc/wireguard/$WG_INTERFACE.conf" ]; then
    WG_PRIV=$(wg genkey)
    WG_PUB=$(echo "$WG_PRIV" | wg pubkey)

    read -rp "Enter VPS public IP or domain: " VPS_ENDPOINT
    read -rp "Enter VPS WireGuard public key: " VPS_PUBKEY

    cat > "/etc/wireguard/$WG_INTERFACE.conf" << WGEOF
[Interface]
# Pi side
Address = 10.0.0.2/24
PrivateKey = $WG_PRIV

[Peer]
# VPS side
PublicKey = $VPS_PUBKEY
Endpoint = $VPS_ENDPOINT:51820
AllowedIPs = 10.0.0.1/32
PersistentKeepalive = 25
WGEOF

    chmod 600 "/etc/wireguard/$WG_INTERFACE.conf"

    echo ""
    echo "  Pi WireGuard public key (paste into VPS's wg0.conf):"
    echo "    $WG_PUB"
    echo ""

    systemctl enable --now "wg-quick@$WG_INTERFACE"
else
    echo "  WireGuard config already exists, skipping."
fi

# ── 3. Pose sync systemd service ────────────────────────────────────
echo "[2/3] Creating pose_sync systemd service..."

read -rp "Enter VPS URL (e.g. http://10.0.0.1:6700 or http://your-domain.com): " VPS_URL

# Get the current real user (not root via sudo)
REAL_USER="${SUDO_USER:-$(whoami)}"
REAL_HOME=$(eval echo "~$REAL_USER")

cat > /etc/systemd/system/warebot-pose-sync.service << EOF
[Unit]
Description=WareBot Pose Sync (Pi → VPS)
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=$REAL_USER
ExecStart=/usr/bin/python3 $WAREBOT_DIR/pi/pose_sync.py \\
    --vps-url $VPS_URL \\
    --db $REAL_HOME/.warebot/pose_history.db \\
    --interval 5
Restart=always
RestartSec=10
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
EOF

systemctl daemon-reload
systemctl enable warebot-pose-sync.service
systemctl start warebot-pose-sync.service

# ── Done ─────────────────────────────────────────────────────────────
echo "[3/3] Done!"
echo ""
echo "════════════════════════════════════════════════════════"
echo "  Pi bridge setup complete!"
echo ""
echo "  To launch the ROS2 bridge nodes:"
echo "    ros2 launch $WAREBOT_DIR/pi/pi_bridge.launch.py"
echo ""
echo "  Or add to your robot launcher. The Pi now runs:"
echo "    - rosbridge_websocket (port 6701)"
echo "    - rosapi"
echo "    - tf_consolidator"
echo "    - service_handler"
echo "    - pose_persistence"
echo "    - pose_sync → pushes to VPS"
echo ""
echo "  Check WireGuard:  sudo wg show"
echo "  Check pose sync:  sudo journalctl -u warebot-pose-sync -f"
echo "════════════════════════════════════════════════════════"
