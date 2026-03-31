#!/usr/bin/env bash
set -e

PIGPIO_VERSION="79"
MEDIAMTX_VERSION="v1.17.0"
HOME_DIR="/home/pi"
MEDIAMTX_DIR="$HOME_DIR/mediamtx"

echo "====================================="
echo "AnglerFish dependency installer"
echo "====================================="

echo
echo "[1/5] Installing apt packages..."
sudo apt update
sudo apt install -y \
    git \
    python3-setuptools \
    python3-full \
    python3-picamera2 \
    python3-smbus \
    python3-pip \
    wget \
    tar \
    build-essential

echo
echo "[2/5] Installing pigpio from source..."
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
echo "[3/5] Installing MediaMTX..."
mkdir -p "$MEDIAMTX_DIR"
cd "$MEDIAMTX_DIR"

MEDIAMTX_ARCHIVE="mediamtx_${MEDIAMTX_VERSION}_linux_arm64.tar.gz"
MEDIAMTX_URL="https://github.com/bluenviron/mediamtx/releases/download/${MEDIAMTX_VERSION}/${MEDIAMTX_ARCHIVE}"

rm -f mediamtx || true
rm -f "$MEDIAMTX_ARCHIVE" || true

wget "$MEDIAMTX_URL"
tar -xzf "$MEDIAMTX_ARCHIVE"
chmod +x mediamtx

echo
echo "Writing MediaMTX config..."
cat > "$MEDIAMTX_DIR/mediamtx.yml" <<'EOF'
logLevel: info

# Disable unused stuff (reduces errors + CPU)
rtmp: no
hls: no
webrtc: no
srt: no

paths:
  cam:
    source: publisher
EOF

echo
echo "[4/5] Installing Python package..."
sudo pip3 install adafruit-circuitpython-ads1x15 --break-system-packages

echo
echo "[5/5] Verifying installs..."
echo "git:      $(command -v git || echo 'NOT FOUND')"
echo "python3:  $(command -v python3 || echo 'NOT FOUND')"
echo "pigpiod:  $(command -v pigpiod || echo 'NOT FOUND')"
echo "mediamtx: $MEDIAMTX_DIR/mediamtx"

if [ -x "$MEDIAMTX_DIR/mediamtx" ]; then
    echo "MediaMTX binary is present and executable."
else
    echo "WARNING: MediaMTX binary missing or not executable."
fi

if [ -f "$MEDIAMTX_DIR/mediamtx.yml" ]; then
    echo "MediaMTX config written successfully."
else
    echo "WARNING: MediaMTX config file was not created."
fi

echo
echo "====================================="
echo "Dependency installation complete"
echo "====================================="
echo
echo "Notes:"
echo "- pigpio can be started manually for troubleshooting with:"
echo "    sudo pigpiod"
echo
echo "- MediaMTX is installed in:"
echo "    $MEDIAMTX_DIR"
echo
echo "- MediaMTX config is:"
echo "    $MEDIAMTX_DIR/mediamtx.yml"
echo
echo "- This script does NOT install or enable any systemd services."