# Autonomous Drone Client

Production-ready autonomous person-tracking drone system for Raspberry Pi 5.

## Features

- **YOLO11 Person Detection**: Real-time person detection using ONNX-optimized YOLO11n
- **Autonomous Tracking**: SPF-inspired 2D→3D command transformation with PID control
- **TOF Sensor Integration**: Scalable TCA9548A multiplexer support (2-8 sensors)
- **Safety Features**: RC override detection, battery monitoring, lost target handling
- **Web Interface**: Real-time telemetry and control via WebSocket
- **Production Ready**: Systemd service, auto-start, comprehensive logging

## Hardware Requirements

- Raspberry Pi 5 (8GB recommended)
- SpeedyBee F405 V4 flight controller
- RPi Camera Module 3 (Wide)
- 2x VL53L1X TOF sensors
- TCA9548A I2C multiplexer
- X1200 UPS HAT (optional but recommended)

## Quick Start

### 1. Setup RPi OS Lite

```bash
# Run setup script
sudo bash rpi/scripts/setup_rpi_lite.sh
```

### 2. Install Dependencies

```bash
# Install Python dependencies
bash rpi/scripts/install_dependencies.sh
```

### 3. Configure System

```bash
# Copy configuration
cp rpi/drone_client/config/production.yaml /home/pi/drone_client/config/

# Setup systemd service
sudo bash rpi/scripts/setup-service.sh
```

### 4. Test Hardware

```bash
# Test all hardware components
python3 rpi/scripts/test_hardware.py

# Test software components
python3 rpi/scripts/test_components.py
```

### 5. Start Service

```bash
# Start drone client
sudo systemctl start drone-client

# Check status
sudo systemctl status drone-client

# View logs
sudo journalctl -u drone-client -f
```

## Configuration

Edit `/home/pi/drone_client/config/production.yaml` to customize:

- **Distance Modes**: Close (2m), Medium (4m), Far (6m)
- **Safety Thresholds**: Battery warnings, auto-land levels
- **PID Gains**: Tune tracking behavior
- **TOF Sensors**: Add/remove sensors as needed

## Usage

### Web Interface

1. Open your deployed frontend
2. Connect to drone via WebSocket
3. Select distance mode
4. Click "Follow" on detected person
5. Monitor telemetry and status

### Manual Override

- Move RC sticks to take manual control
- System automatically pauses autonomous mode
- Return sticks to neutral to resume

### Safety Features

- **Battery Monitoring**: Auto-land at 15%
- **Lost Target**: Hover after 2s, land after 10s
- **RC Override**: Instant manual control
- **TOF Obstacle Avoidance**: Forward and downward sensors

## Architecture

```
rpi/drone_client/
├── main.py                    # Main orchestrator
├── config.py                  # Configuration management
├── controllers/               # Flight control
│   ├── mavlink_controller.py  # MAVLink interface
│   ├── tracking_controller.py # Vision→commands
│   └── pid_controller.py      # PID control
├── vision/                    # Computer vision
│   ├── yolo_detector.py       # YOLO11 detection
│   ├── person_tracker.py      # ID tracking
│   ├── depth_estimator.py     # Distance estimation
│   └── vlm_selector.py        # VLM (optional)
├── sensors/                   # Hardware interfaces
│   ├── camera.py              # Picamera2 wrapper
│   ├── tof_manager.py         # TOF sensors
│   └── telemetry.py           # Data collection
├── communication/             # Network
│   └── websocket_client.py    # Backend connection
├── safety/                    # Safety features
│   ├── rc_override_monitor.py # Manual control detection
│   ├── lost_target_handler.py # Target loss handling
│   └── battery_monitor.py     # Battery safety
└── utils/                     # Utilities
    ├── logger.py              # Logging
    └── state_machine.py       # State management
```

## Development

### Testing

```bash
# Test hardware
python3 rpi/scripts/test_hardware.py

# Test components
python3 rpi/scripts/test_components.py

# Run specific tests
python3 -m pytest rpi/tests/
```

### Logging

```bash
# View live logs
sudo journalctl -u drone-client -f

# View log file
tail -f /var/log/drone_client.log
```

### Debugging

```bash
# Run in debug mode
cd /home/pi/drone_client
source venv/bin/activate
python3 -m drone_client.main --debug
```

## Troubleshooting

### Common Issues

1. **MAVLink Connection Failed**
   - Check `/dev/ttyAMA0` permissions
   - Verify baud rate (256000)
   - Ensure flight controller is powered

2. **Camera Not Found**
   - Enable camera in raspi-config
   - Check camera cable connection
   - Verify camera permissions

3. **TOF Sensors Not Working**
   - Check I2C is enabled
   - Verify TCA9548A connections
   - Test with `i2cdetect -y 1`

4. **YOLO Model Not Found**
   - Download YOLO11n.onnx to `/home/pi/models/`
   - Check file permissions
   - Verify model path in config

### Performance Optimization

- Use RPi OS Lite (no GUI)
- Enable GPU memory split
- Use fast SD card (Class 10+)
- Consider USB 3.0 storage

## Contributing

1. Fork the repository
2. Create feature branch
3. Add tests for new features
4. Submit pull request

## License

MIT License - see LICENSE file for details.

## Support

For issues and questions:
- Check logs: `sudo journalctl -u drone-client`
- Run tests: `python3 rpi/scripts/test_hardware.py`
- Review configuration: `/home/pi/drone_client/config/production.yaml`
