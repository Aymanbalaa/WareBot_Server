#!/usr/bin/env bash
# ──────────────────────────────────────────────────────────────────────
# WareBot VPS Setup Script  –  Ubuntu 22.04
# ──────────────────────────────────────────────────────────────────────
# Run as root or with sudo.
#
# What this does:
#   1. Installs system deps (nginx, python3, pip, wireguard)
#   2. Clones the repo (or uses existing)
#   3. Creates a Python venv and installs Flask + Waitress
#   4. Installs the Nginx config
#   5. Creates a systemd unit for the Flask server
#   6. Sets up WireGuard (interactive – needs Pi's public key)
#
# Usage:
#   chmod +x setup_vps.sh
#   sudo ./setup_vps.sh
# ──────────────────────────────────────────────────────────────────────

set -euo pipefail

REPO_URL="https://github.com/Aymanbalaa/WareBot_Server.git"
INSTALL_DIR="/opt/warebot"
VENV_DIR="$INSTALL_DIR/venv"
SERVICE_USER="warebot"
WG_INTERFACE="wg0"

echo "════════════════════════════════════════════════════════"
echo "  WareBot VPS Setup  –  Ubuntu 22.04"
echo "════════════════════════════════════════════════════════"

# ── 1. System packages ───────────────────────────────────────────────
echo "[1/6] Installing system packages..."
apt-get update -qq
apt-get install -y -qq \
    python3 python3-pip python3-venv \
    nginx \
    wireguard \
    git \
    certbot python3-certbot-nginx \
    > /dev/null

# ── 2. Clone / update repo ──────────────────────────────────────────
echo "[2/6] Setting up repository..."
if [ -d "$INSTALL_DIR/.git" ]; then
    echo "  Repo exists, pulling latest..."
    cd "$INSTALL_DIR"
    git pull --ff-only
else
    echo "  Cloning into $INSTALL_DIR..."
    git clone "$REPO_URL" "$INSTALL_DIR"
fi

# ── 3. Python venv + deps ───────────────────────────────────────────
echo "[3/6] Setting up Python environment..."
python3 -m venv "$VENV_DIR"
"$VENV_DIR/bin/pip" install --upgrade pip -q
"$VENV_DIR/bin/pip" install -r "$INSTALL_DIR/vps/requirements.txt" -q

# ── 4. System user ──────────────────────────────────────────────────
if ! id "$SERVICE_USER" &>/dev/null; then
    echo "  Creating system user '$SERVICE_USER'..."
    useradd --system --home-dir "$INSTALL_DIR" --shell /usr/sbin/nologin "$SERVICE_USER"
fi
mkdir -p "/home/$SERVICE_USER/.warebot"
chown -R "$SERVICE_USER:$SERVICE_USER" "$INSTALL_DIR" "/home/$SERVICE_USER"

# ── 5. Nginx ─────────────────────────────────────────────────────────
echo "[4/6] Configuring Nginx..."
cp "$INSTALL_DIR/vps/warebot.nginx.conf" /etc/nginx/sites-available/warebot
ln -sf /etc/nginx/sites-available/warebot /etc/nginx/sites-enabled/
rm -f /etc/nginx/sites-enabled/default

# Remind to edit PI_WG_IP
if grep -q "PI_WG_IP" /etc/nginx/sites-available/warebot; then
    echo ""
    echo "  ⚠  You MUST edit /etc/nginx/sites-available/warebot"
    echo "     and replace PI_WG_IP with the Pi's WireGuard IP (e.g. 10.0.0.2)"
    echo ""
fi

nginx -t && systemctl reload nginx

# ── 6. Systemd units ────────────────────────────────────────────────
echo "[5/6] Creating systemd services..."

cat > /etc/systemd/system/warebot-web.service << 'EOF'
[Unit]
Description=WareBot Flask Web Server
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=warebot
Group=warebot
WorkingDirectory=/opt/warebot
ExecStart=/opt/warebot/venv/bin/python3 /opt/warebot/vps/server.py --config /opt/warebot/vps/config.json
Restart=always
RestartSec=5
StandardOutput=journal
StandardError=journal

# Hardening
NoNewPrivileges=true
ProtectSystem=strict
ReadWritePaths=/home/warebot/.warebot

[Install]
WantedBy=multi-user.target
EOF

systemctl daemon-reload
systemctl enable warebot-web.service
systemctl start warebot-web.service

# ── 7. WireGuard scaffold ───────────────────────────────────────────
echo "[6/6] WireGuard setup..."

if [ ! -f "/etc/wireguard/$WG_INTERFACE.conf" ]; then
    # Generate keys
    WG_PRIV=$(wg genkey)
    WG_PUB=$(echo "$WG_PRIV" | wg pubkey)

    cat > "/etc/wireguard/$WG_INTERFACE.conf" << WGEOF
[Interface]
# VPS side
Address = 10.0.0.1/24
ListenPort = 51820
PrivateKey = $WG_PRIV

[Peer]
# Pi side  –  paste the Pi's public key here
PublicKey = PASTE_PI_PUBLIC_KEY_HERE
AllowedIPs = 10.0.0.2/32
PersistentKeepalive = 25
WGEOF

    chmod 600 "/etc/wireguard/$WG_INTERFACE.conf"

    echo ""
    echo "  WireGuard VPS public key (give this to Pi):"
    echo "    $WG_PUB"
    echo ""
    echo "  Edit /etc/wireguard/$WG_INTERFACE.conf and paste Pi's public key,"
    echo "  then:  sudo systemctl enable --now wg-quick@$WG_INTERFACE"
else
    echo "  WireGuard config already exists, skipping."
fi

# ── Done ─────────────────────────────────────────────────────────────
echo ""
echo "════════════════════════════════════════════════════════"
echo "  Setup complete!"
echo ""
echo "  Next steps:"
echo "    1. Edit /etc/nginx/sites-available/warebot"
echo "       → replace PI_WG_IP with 10.0.0.2 (or Pi's WG IP)"
echo "    2. Edit /opt/warebot/vps/config.json"
echo "       → set port_rosbridge to 80 (browser connects via Nginx)"
echo "       → or keep 6701 if using direct connection"
echo "    3. Set up WireGuard on the Pi (run pi/setup_pi.sh)"
echo "    4. sudo systemctl enable --now wg-quick@wg0"
echo "    5. sudo systemctl reload nginx"
echo "    6. Optional: sudo certbot --nginx -d your-domain.com"
echo ""
echo "  Check status:"
echo "    sudo systemctl status warebot-web"
echo "    sudo journalctl -u warebot-web -f"
echo "════════════════════════════════════════════════════════"
