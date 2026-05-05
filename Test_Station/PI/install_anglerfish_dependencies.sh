#!/usr/bin/env bash
set -e

PIGPIO_VERSION="79"
HOME_DIR="/home/pi"

echo "====================================="
echo "AnglerFish dependency installer"
echo "====================================="

echo
echo "[1/4] Installing apt packages..."
sudo apt update
sudo apt install -y \
    git \
    python3 \
    python3-setuptools \
    python3-full \
    python3-picamera2 \
    python3-smbus \
    python3-pip \
    libcamera-apps \
    gstreamer1.0-tools \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    wget \
    tar \
    build-essential

echo
echo "[2/4] Installing pigpio from source..."
cd "$HOME_DIR"

rm -rf "pigpio-$PIGPIO_VERSION" || true
rm -f "v${PIGPIO_VERSION}.tar.gz" || true

wget "https://github.com/joan2937/pigpio/archive/refs/tags/v${PIGPIO_VERSION}.tar.gz"
tar zxf "v${PIGPIO_VERSION}.tar.gz"
cd "pigpio-${PIGPIO_VERSION}"
make
sudo make install
sudo ldconfig
sudo systemctl daemon-reload

PIGPIOD_PATH="$(command -v pigpiod || true)"
echo
if [ -n "$PIGPIOD_PATH" ]; then
    echo "pigpiod found at: $PIGPIOD_PATH"
else
    echo "WARNING: pigpiod was not found in PATH after install."
    echo "You may need to locate it manually later."
fi

echo
echo "[3/4] Installing Python package..."
sudo pip3 install adafruit-circuitpython-ads1x15 --break-system-packages

echo
echo "[4/4] Verifying installs..."
echo "git:      $(command -v git || echo 'NOT FOUND')"
echo "python3:  $(command -v python3 || echo 'NOT FOUND')"
echo "pigpiod:  $(command -v pigpiod || echo 'NOT FOUND')"
echo "rpicam-vid: $(command -v rpicam-vid || echo 'NOT FOUND')"
echo "gst-launch-1.0: $(command -v gst-launch-1.0 || echo 'NOT FOUND')"

echo
echo "====================================="
echo "Dependency installation complete"
echo "====================================="
echo
echo "Notes:"
echo "- pigpio can be started manually for troubleshooting with:"
echo "    sudo pigpiod"
echo
echo "- This script does NOT install or enable any systemd services."