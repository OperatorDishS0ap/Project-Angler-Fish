#!/usr/bin/env bash
set -e

SERVICE_DIR="/etc/systemd/system"
ANGLERFISH_DIR="/home/pi/anglerfish"

# Adjust this if pigpiod is elsewhere
PIGPIOD_PATH="/usr/local/bin/pigpiod"

echo "Creating AnglerFish systemd files..."

sudo tee "${SERVICE_DIR}/anglerfish.target" > /dev/null <<'EOF'
[Unit]
Description=AnglerFish Full System
Wants=network-online.target pigpiod.service anglerfish-camera.service anglerfish-sensors.service anglerfish-motors.service
After=network-online.target

[Install]
WantedBy=multi-user.target
EOF

sudo tee "${SERVICE_DIR}/pigpiod.service" > /dev/null <<EOF
[Unit]
Description=Pigpio Daemon
After=network-online.target
Wants=network-online.target
PartOf=anglerfish.target

[Service]
Type=forking
ExecStart=${PIGPIOD_PATH}
Restart=on-failure
RestartSec=3

[Install]
WantedBy=anglerfish.target
EOF

sudo tee "${SERVICE_DIR}/anglerfish-camera.service" > /dev/null <<EOF
[Unit]
Description=AnglerFish Camera Stream
After=network-online.target
Wants=network-online.target
PartOf=anglerfish.target

[Service]
Type=simple
User=pi
WorkingDirectory=${ANGLERFISH_DIR}
ExecStart=/usr/bin/python3 ${ANGLERFISH_DIR}/camera.py
Restart=on-failure
RestartSec=3
Environment=PYTHONUNBUFFERED=1

[Install]
WantedBy=anglerfish.target
EOF

sudo tee "${SERVICE_DIR}/anglerfish-sensors.service" > /dev/null <<EOF
[Unit]
Description=AnglerFish Sensor Telemetry
After=anglerfish-camera.service
Wants=anglerfish-camera.service
PartOf=anglerfish.target

[Service]
Type=simple
User=pi
WorkingDirectory=${ANGLERFISH_DIR}
ExecStart=/usr/bin/python3 ${ANGLERFISH_DIR}/sensors.py
Restart=on-failure
RestartSec=3
Environment=PYTHONUNBUFFERED=1

[Install]
WantedBy=anglerfish.target
EOF

sudo tee "${SERVICE_DIR}/anglerfish-motors.service" > /dev/null <<EOF
[Unit]
Description=AnglerFish Motor Controller
After=anglerfish-sensors.service pigpiod.service
Requires=pigpiod.service
Wants=anglerfish-sensors.service
PartOf=anglerfish.target

[Service]
Type=simple
User=pi
WorkingDirectory=${ANGLERFISH_DIR}
ExecStart=/usr/bin/python3 ${ANGLERFISH_DIR}/motors.py
Restart=on-failure
RestartSec=3
Environment=PYTHONUNBUFFERED=1

[Install]
WantedBy=anglerfish.target
EOF

echo "Reloading systemd..."
sudo systemctl daemon-reload

echo "Disabling standalone boot entries..."
sudo systemctl disable anglerfish-camera.service 2>/dev/null || true
sudo systemctl disable anglerfish-motors.service 2>/dev/null || true
sudo systemctl disable anglerfish-sensors.service 2>/dev/null || true
sudo systemctl disable pigpiod.service 2>/dev/null || true

echo "Enabling master target..."
sudo systemctl enable anglerfish.target

echo "Done."
echo
echo "To start everything now:"
echo "  sudo systemctl start anglerfish.target"
echo
echo "To stop everything:"
echo "  sudo systemctl stop anglerfish.target"
echo
echo "To view status:"
  echo "  systemctl list-units --type=service | grep -E 'anglerfish|pigpiod'"