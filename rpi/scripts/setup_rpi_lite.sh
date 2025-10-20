#!/bin/bash
# Setup script for RPi OS Lite with drone client

set -e

echo "Setting up RPi OS Lite for autonomous drone client..."

# Update system
sudo apt update && sudo apt upgrade -y

# Install system dependencies
sudo apt install -y \
    python3 \
    python3-pip \
    python3-venv \
    python3-dev \
    git \
    curl \
    wget \
    vim \
    htop \
    i2c-tools \
    libi2c-dev \
    libopencv-dev \
    python3-opencv \
    libatlas-base-dev \
    libhdf5-dev \
    libhdf5-serial-dev \
    libqtgui4 \
    libqtwebkit4 \
    libqt4-test \
    python3-pyqt5 \
    libblas-dev \
    liblapack-dev \
    libhdf5-dev \
    pkg-config

# Enable I2C
echo "Enabling I2C interface..."
sudo raspi-config nonint do_i2c 0

# Enable camera
echo "Enabling camera interface..."
sudo raspi-config nonint do_camera 0

# Enable SSH
echo "Enabling SSH..."
sudo systemctl enable ssh
sudo systemctl start ssh

# Create drone client directory
sudo mkdir -p /home/pi/drone_client
sudo chown pi:pi /home/pi/drone_client

# Create models directory
sudo mkdir -p /home/pi/models
sudo chown pi:pi /home/pi/models

# Create log directory
sudo mkdir -p /var/log
sudo touch /var/log/drone_client.log
sudo chown pi:pi /var/log/drone_client.log

echo "RPi OS Lite setup complete!"
echo "Next steps:"
echo "1. Copy drone client code to /home/pi/drone_client"
echo "2. Run install_dependencies.sh"
echo "3. Run setup-service.sh"
