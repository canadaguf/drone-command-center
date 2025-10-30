# Raspberry Pi Connection Test Scripts

This directory contains test scripts to verify your Raspberry Pi can connect to the drone's flight controller and read heartbeat messages.

## Quick Start

1. **First, configure serial port** (see `SERIAL_SETUP.md` for details):
   ```bash
   sudo raspi-config
   # Disable Serial login shell
   # Enable Serial interface
   sudo reboot
   ```

2. **Add yourself to dialout group**:
   ```bash
   sudo usermod -a -G dialout $USER
   # Logout and login again
   ```

3. **Run the simple test**:
   ```bash
   python3 simple_heartbeat_test.py
   ```

4. **If that works, run comprehensive test**:
   ```bash
   python3 test_connection.py
   ```

## Test Scripts

### `simple_heartbeat_test.py`
**Purpose:** Minimal test to verify basic connection and heartbeat reception.

**Usage:**
```bash
python3 simple_heartbeat_test.py
```

**What it does:**
- Tries multiple serial ports (`/dev/ttyAMA0`, `/dev/serial0`)
- Tries multiple baud rates (256000, 115200)
- Stops on first successful connection
- Shows basic connection info

**Output:** Simple pass/fail with working configuration

### `test_connection.py`
**Purpose:** Comprehensive diagnostic test with detailed information.

**Usage:**
```bash
python3 test_connection.py
```

**What it does:**
- Checks serial port existence and permissions
- Verifies user is in dialout group
- Tests all port/baud combinations
- Shows detailed heartbeat information:
  - System/Component IDs
  - Vehicle type (Quadrotor, Hexarotor, etc.)
  - Autopilot type (ArduPilot, PX4)
  - Armed status
  - Flight mode
- Provides troubleshooting guidance if connection fails

**Output:** Detailed diagnostics and troubleshooting steps

### `worked_best/` Directory
Contains previously working test snippets:
- `heart_check.py` - Simple heartbeat check (115200 baud)
- `basic_config_check.py` - Full system configuration check (256000 baud)
- `basic_arm.py` - Arm/disarm test (256000 baud)
- `gps_check.py` - GPS data check
- `drone_client.py` - WebSocket client example

## Common Issues

### "Port not found"
- Enable serial interface: `sudo raspi-config` → Interface Options → Serial Port
- Check `/boot/config.txt` has `enable_uart=1`
- Reboot after changes

### "Permission denied"
- Add user to dialout: `sudo usermod -a -G dialout $USER`
- Logout and login again

### "No heartbeat received"
- Check physical connections (TX/RX/GND)
- Verify flight controller is powered on
- Check serial console is disabled (see `SERIAL_SETUP.md`)
- Try different baud rates
- Check if port is in use: `sudo lsof /dev/ttyAMA0`

## Configuration Files

The scripts automatically try common configurations:
- **Ports:** `/dev/ttyAMA0`, `/dev/serial0`
- **Baud rates:** 256000, 115200

For production use, the drone client uses:
- Port: `/dev/ttyAMA0` (configurable in `drone_client/config/production.yaml`)
- Baud: 256000 (configurable in `drone_client/config/production.yaml`)

## Next Steps

Once heartbeat is working:
1. ✅ Connection verified
2. Test with `basic_config_check.py` to see full system info
3. Test with `basic_arm.py` to verify arm/disarm (be careful!)
4. Configure and run full drone client system

## Camera Tests

### `simple_camera_test.py`
**Purpose:** Quick camera test to verify Picamera2 works.

**Usage:**
```bash
python3 simple_camera_test.py
```

**What it does:**
- Checks numpy and picamera2 imports
- Initializes camera (640x480 @ 30fps)
- Captures 3 test frames
- Shows frame properties

**Prerequisites:**
- System packages: `sudo apt install -y python3-picamera2 libcamera-dev libcap-dev`
- Camera enabled: `sudo raspi-config → Interface Options → Camera`
- Python package: `pip install picamera2 numpy`

### `test_camera.py`
**Purpose:** Comprehensive camera diagnostics with detailed information.

**Usage:**
```bash
python3 test_camera.py
```

**What it does:**
- Step-by-step import verification
- Camera detection and model info
- Frame capture testing
- Performance metrics (FPS estimation)
- Detailed troubleshooting guidance

**Output:** Detailed diagnostics and frame statistics

## See Also

- `SERIAL_SETUP.md` - Detailed serial port configuration guide
- `CAMERA_SETUP.md` - Camera setup and troubleshooting guide
- `CONNECTION_WORKING.md` - Working connection configuration notes
- `../README.md` - Main project documentation


