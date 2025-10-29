#!/bin/bash
# One-shot bootstrap for Raspberry Pi OS Lite (64-bit) to run the drone client

set -euo pipefail

# Defaults (can be overridden via env vars or flags)
REPO_DIR_DEFAULT="$HOME/drone-command-center"
CLIENT_DST_DIR="/home/pi/drone_client"
MODELS_DIR="/home/pi/models"
LOG_FILE="/var/log/drone_client.log"
PY_VENV_DIR="$CLIENT_DST_DIR/venv"
MODEL_URL_DEFAULT=""  # set via --model-url to auto-download
SETUP_SYSTEMD=false

usage() {
  echo "Usage: sudo bash rpi/scripts/bootstrap_pi.sh [--repo <path>] [--model-url <url>] [--setup-systemd]"
  echo "  --repo <path>        Path to local repo (default: $REPO_DIR_DEFAULT)"
  echo "  --model-url <url>    Optional URL to download YOLO11n.onnx to $MODELS_DIR"
  echo "  --setup-systemd      Install and enable the systemd service"
}

REPO_DIR="$REPO_DIR_DEFAULT"
MODEL_URL="$MODEL_URL_DEFAULT"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --repo)
      REPO_DIR="$2"; shift 2;;
    --model-url)
      MODEL_URL="$2"; shift 2;;
    --setup-systemd)
      SETUP_SYSTEMD=true; shift;;
    -h|--help)
      usage; exit 0;;
    *)
      echo "Unknown argument: $1"; usage; exit 1;;
  esac
done

echo "=== Drone Client Bootstrap ==="
echo "Repo dir:        $REPO_DIR"
echo "Client dst dir:  $CLIENT_DST_DIR"
echo "Models dir:      $MODELS_DIR"
echo "Venv dir:        $PY_VENV_DIR"
echo "Model URL:       ${MODEL_URL:-<none>}"
echo "Setup systemd:   $SETUP_SYSTEMD"

# 0) Basic sanity checks
if [[ $(id -u) -ne 0 ]]; then
  echo "Please run with sudo (this script configures system packages and services)." >&2
  exit 1
fi

if [[ ! -d "$REPO_DIR" ]]; then
  echo "Repo directory not found: $REPO_DIR" >&2
  echo "Clone it first, e.g.: git clone <repo_url> $REPO_DIR" >&2
  exit 1
fi

# 1) System update and core packages
echo "[1/6] Updating system and installing apt packages..."
apt update
DEBIAN_FRONTEND=noninteractive apt full-upgrade -y

apt install -y \
  python3 python3-pip python3-venv python3-dev \
  git curl wget htop \
  i2c-tools libi2c-dev pkg-config \
  libatlas-base-dev libblas-dev liblapack-dev libhdf5-dev \
  python3-opencv \
  python3-picamera2 libcamera-dev libcap-dev

# 2) Enable interfaces and SSH
echo "[2/6] Enabling I2C, camera, and SSH..."
raspi-config nonint do_i2c 0 || true
raspi-config nonint do_camera 0 || true
systemctl enable ssh --now || true

# 3) Create directories and log file
echo "[3/6] Creating directories and setting ownership..."
mkdir -p "$CLIENT_DST_DIR" "$MODELS_DIR"
touch "$LOG_FILE"
chown -R pi:pi "$CLIENT_DST_DIR" "$MODELS_DIR"
chown pi:pi "$LOG_FILE"

# 4) Python venv and pip dependencies
echo "[4/6] Creating Python virtual environment and installing dependencies..."
sudo -u pi bash -lc "python3 -m venv '$PY_VENV_DIR'"
sudo -u pi bash -lc "source '$PY_VENV_DIR/bin/activate' && pip install --upgrade pip"
sudo -u pi bash -lc "source '$PY_VENV_DIR/bin/activate' && pip install fastapi uvicorn websockets pymavlink numpy pyyaml smbus2 RPi.GPIO onnxruntime"

# 5) Copy code and config
echo "[5/6] Copying drone client code and config..."
rsync -a --delete "$REPO_DIR/rpi/drone_client/" "$CLIENT_DST_DIR/"
mkdir -p "$CLIENT_DST_DIR/config"
if [[ -f "$REPO_DIR/rpi/drone_client/config/production.yaml" ]]; then
  cp "$REPO_DIR/rpi/drone_client/config/production.yaml" "$CLIENT_DST_DIR/config/"
fi
chown -R pi:pi "$CLIENT_DST_DIR"

# 6) Optional: download model
if [[ -n "$MODEL_URL" ]]; then
  echo "[6/6] Downloading model to $MODELS_DIR ..."
  sudo -u pi bash -lc "wget -O '$MODELS_DIR/yolo11n.onnx' '$MODEL_URL'"
fi

# Optional: systemd setup
if [[ "$SETUP_SYSTEMD" == "true" ]]; then
  echo "Setting up systemd service..."
  if [[ -f "$REPO_DIR/rpi/scripts/setup-service.sh" ]]; then
    bash "$REPO_DIR/rpi/scripts/setup-service.sh"
  else
    echo "setup-service.sh not found at $REPO_DIR/rpi/scripts/setup-service.sh" >&2
    exit 1
  fi
fi

echo "=== Bootstrap complete ==="
echo "Next steps:"
echo "  1) Validate camera: libcamera-hello (should display preview if a display is attached)"
echo "  2) Validate I2C: i2cdetect -y 1 (should show multiplexer/sensors)"
echo "  3) Activate venv: source $PY_VENV_DIR/bin/activate"
echo "  4) Test: python3 $REPO_DIR/rpi/scripts/test_hardware.py && python3 $REPO_DIR/rpi/scripts/test_components.py"
echo "  5) Start service: sudo systemctl start drone-client (if installed)"


