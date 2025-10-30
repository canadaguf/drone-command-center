#!/usr/bin/env python3
"""
Camera hardware diagnostic - check if camera is detected at system level
Run this BEFORE trying picamera2 to verify camera hardware is working
"""

import subprocess
import sys
import os

print("=" * 60)
print("Camera Hardware Diagnostic")
print("=" * 60)

# Check 1: libcamera-hello (system camera test)
print("\n[1/3] Testing camera with libcamera-hello...")
try:
    result = subprocess.run(
        ['libcamera-hello', '--timeout', '2000'],
        capture_output=True,
        text=True,
        timeout=5
    )
    if result.returncode == 0:
        print("  ✓ libcamera-hello succeeded - camera hardware is working!")
    else:
        print(f"  ❌ libcamera-hello failed (exit code: {result.returncode})")
        if result.stderr:
            print(f"     Error: {result.stderr[:200]}")
except FileNotFoundError:
    print("  ⚠️  libcamera-hello not found")
    print("     Install with: sudo apt install -y libcamera-apps")
except subprocess.TimeoutExpired:
    print("  ⚠️  libcamera-hello timed out (camera may be working but slow)")
except Exception as e:
    print(f"  ⚠️  Error running libcamera-hello: {e}")

# Check 2: List cameras
print("\n[2/3] Listing available cameras...")
try:
    result = subprocess.run(
        ['libcamera-hello', '--list-cameras'],
        capture_output=True,
        text=True,
        timeout=5
    )
    if result.returncode == 0:
        if result.stdout:
            print("  ✓ Cameras found:")
            print(result.stdout)
        else:
            print("  ⚠️  No camera output (might still be OK)")
    else:
        print(f"  ⚠️  Could not list cameras: {result.stderr[:200]}")
except Exception as e:
    print(f"  ⚠️  Error listing cameras: {e}")

# Check 3: Check /dev/video* devices
print("\n[3/3] Checking /dev/video* devices...")
video_devices = []
for i in range(10):
    device = f"/dev/video{i}"
    if os.path.exists(device):
        video_devices.append(device)
        # Get device info
        try:
            import stat
            st = os.stat(device)
            mode = stat.filemode(st.st_mode)
            print(f"  ✓ Found: {device} ({mode})")
        except:
            print(f"  ✓ Found: {device}")

if not video_devices:
    print("  ⚠️  No /dev/video* devices found")
    print("     This might indicate camera not detected")
else:
    print(f"  ✓ Found {len(video_devices)} video device(s)")

# Summary
print("\n" + "=" * 60)
print("Summary")
print("=" * 60)

# Check if camera interface is enabled
try:
    with open('/boot/config.txt', 'r') as f:
        config_content = f.read()
        if 'camera_auto_detect=1' in config_content or 'start_x=1' in config_content or 'gpu_mem=' in config_content:
            print("✓ Camera interface appears to be enabled in /boot/config.txt")
        else:
            print("⚠️  Camera interface may not be enabled in /boot/config.txt")
            print("   Enable with: sudo raspi-config → Interface Options → Camera")
except Exception as e:
    print(f"⚠️  Could not check /boot/config.txt: {e}")

print("\nNext steps:")
print("1. If libcamera-hello works → Camera hardware is OK, try picamera2 test")
print("2. If libcamera-hello fails → Check camera connection and enable interface")
print("3. Run: python3 simple_camera_test.py")

