#!/bin/bash
# Setup systemd service for drone client

set -e

echo "Setting up systemd service..."

# Copy service file
sudo cp /home/pi/drone_client/systemd/drone-client.service /etc/systemd/system/

# Reload systemd
sudo systemctl daemon-reload

# Enable service
sudo systemctl enable drone-client.service

echo "Systemd service setup complete!"
echo "Service commands:"
echo "  sudo systemctl start drone-client    # Start service"
echo "  sudo systemctl stop drone-client     # Stop service"
echo "  sudo systemctl status drone-client   # Check status"
echo "  sudo systemctl logs drone-client     # View logs"
