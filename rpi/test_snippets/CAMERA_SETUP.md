# Camera Setup Guide for Raspberry Pi OS Lite

## Overview
This guide helps you set up and test the Raspberry Pi Camera Module on Raspberry Pi OS Lite.

## Quick Setup Steps

### 1. Install System Packages (REQUIRED)

Picamera2 requires system packages to be installed first:

```bash
sudo apt update
sudo apt install -y python3-picamera2 libcamera-dev libcap-dev
```

**Important:** These must be installed BEFORE installing the Python package via pip.

### 2. Enable Camera Interface

Enable the camera interface using raspi-config:

```bash
sudo raspi-config
```

Navigate to:
- **Interface Options** → **Camera** → **Enable**

**Reboot after enabling:**
```bash
sudo reboot
```

### 3. Verify Camera Hardware

Test camera with system tool:

```bash
libcamera-hello
```

If this works, your camera hardware is correctly connected.

### 4. Install Python Package

If using a virtual environment, you have two options:

**Option A: Use system-site-packages (Recommended)**
```bash
python3 -m venv --system-site-packages venv
source venv/bin/activate
pip install picamera2
```

**Option B: Install in regular venv**
```bash
python3 -m venv venv
source venv/bin/activate
pip install picamera2
```

**Note:** Option A is recommended because picamera2 needs access to system libraries.

### 5. Install Additional Dependencies

```bash
pip install numpy
```

## Testing

Run the test scripts:

```bash
# Simple test
python3 simple_camera_test.py

# Comprehensive test
python3 test_camera.py
```

## Common Import Issues

### Issue 1: "No module named 'picamera2'"

**Cause:** Python package not installed

**Solution:**
```bash
pip install picamera2
```

### Issue 2: Import Error or Missing Libraries

**Cause:** System packages not installed

**Solution:**
```bash
sudo apt install -y python3-picamera2 libcamera-dev libcap-dev
# Reboot may be required
sudo reboot
```

### Issue 3: "Camera not available" or "Failed to initialize"

**Causes:**
- Camera interface not enabled
- Camera not connected properly
- Permissions issue

**Solutions:**
1. Enable camera in raspi-config:
   ```bash
   sudo raspi-config → Interface Options → Camera → Enable
   sudo reboot
   ```

2. Check camera connection (cable seated properly)

3. Test with system tool:
   ```bash
   libcamera-hello
   ```

4. Check permissions (usually not an issue, but verify):
   ```bash
   groups  # Should include your user
   ```

### Issue 4: Import works but camera initialization fails

**Cause:** Camera may be in use by another process

**Solution:**
```bash
# Check if camera is in use
sudo lsof /dev/video*

# Kill any processes using camera
sudo pkill -f libcamera
```

### Issue 5: "No camera detected" in virtual environment

**Cause:** Virtual environment not created with system-site-packages

**Solution:**
```bash
# Recreate venv with system-site-packages
deactivate
rm -rf venv
python3 -m venv --system-site-packages venv
source venv/bin/activate
pip install picamera2 numpy
```

## Installation Order Matters

For a clean setup, follow this order:

1. **Install system packages first:**
   ```bash
   sudo apt install -y python3-picamera2 libcamera-dev libcap-dev
   ```

2. **Enable camera interface:**
   ```bash
   sudo raspi-config → Interface Options → Camera → Enable
   sudo reboot
   ```

3. **Create virtual environment (with system-site-packages):**
   ```bash
   python3 -m venv --system-site-packages venv
   source venv/bin/activate
   ```

4. **Install Python packages:**
   ```bash
   pip install --upgrade pip
   pip install picamera2 numpy
   ```

## Production Configuration

The camera is configured in `drone_client/config/production.yaml`:

```yaml
camera:
  width: 640
  height: 480
  fps: 30
  fov_horizontal: 120
  fov_vertical: 90
```

## Troubleshooting Commands

```bash
# Check if camera is detected
libcamera-hello

# List available cameras
libcamera-hello --list-cameras

# Check camera device
ls -l /dev/video*

# Check installed packages
dpkg -l | grep picamera
dpkg -l | grep libcamera

# Check Python package
pip show picamera2

# Test import
python3 -c "from picamera2 import Picamera2; print('OK')"
```

## Next Steps

Once camera test passes:
1. ✅ Camera verified
2. Test with vision pipeline (YOLO detection)
3. Integrate with drone client system

## See Also

- `simple_camera_test.py` - Quick camera test
- `test_camera.py` - Comprehensive camera diagnostics
- `../README.md` - Main project documentation

