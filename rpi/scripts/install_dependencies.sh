#!/bin/bash
# Install Python dependencies for drone client

set -e

echo "Installing Python dependencies..."

# Create virtual environment
cd /home/pi/drone_client
python3 -m venv venv
source venv/bin/activate

# Upgrade pip
pip install --upgrade pip

# Install dependencies
pip install \
    fastapi \
    uvicorn \
    websockets \
    pymavlink \
    opencv-python \
    numpy \
    pyyaml \
    smbus2 \
    RPi.GPIO \
    picamera2 \
    ultralytics \
    onnxruntime

# Install additional dependencies for YOLO
pip install \
    torch \
    torchvision \
    torchaudio \
    --index-url https://download.pytorch.org/whl/cpu

echo "Dependencies installed successfully!"
echo "Virtual environment created at /home/pi/drone_client/venv"
