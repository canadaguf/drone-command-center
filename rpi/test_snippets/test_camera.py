#!/usr/bin/env python3
"""
Camera test script for Raspberry Pi
Tests Picamera2 initialization, frame capture, and basic functionality
Based on working patterns from the project
"""

import sys
import time

# Check Python version
print("=" * 60)
print("Raspberry Pi Camera Test")
print("=" * 60)
print(f"Python version: {sys.version}")

# Step 1: Check and import numpy (required by picamera2)
print("\n[1/5] Checking numpy...")
try:
    import numpy as np
    print(f"  ✓ numpy {np.__version__} imported successfully")
except ImportError as e:
    print(f"  ❌ Failed to import numpy: {e}")
    print("  Fix: pip install numpy")
    sys.exit(1)

# Step 2: Check and import picamera2 (with detailed error handling)
print("\n[2/5] Checking picamera2...")
try:
    from picamera2 import Picamera2
    print("  ✓ picamera2 imported successfully")
except ImportError as e:
    print(f"  ❌ Failed to import picamera2: {e}")
    print("\n  Troubleshooting:")
    print("  1. Install system packages:")
    print("     sudo apt update")
    print("     sudo apt install -y python3-picamera2 libcamera-dev libcap-dev")
    print("  2. Install Python package:")
    print("     pip install picamera2")
    print("  3. If using venv, create with system-site-packages:")
    print("     python3 -m venv --system-site-packages venv")
    print("  4. Enable camera interface:")
    print("     sudo raspi-config")
    print("     → Interface Options → Camera → Enable")
    print("     → Reboot")
    sys.exit(1)
except Exception as e:
    print(f"  ❌ Unexpected error importing picamera2: {e}")
    print("  This might indicate missing system libraries.")
    print("  Try: sudo apt install -y python3-picamera2 libcamera-dev libcap-dev")
    sys.exit(1)

# Step 3: Check camera availability
print("\n[3/5] Checking camera availability...")
try:
    # Try to get camera info (this will fail if camera not available)
    picam2 = Picamera2()
    camera_info = picam2.camera_properties
    print("  ✓ Camera detected")
    
    # Show camera model if available
    if 'Model' in camera_info:
        print(f"  ✓ Camera Model: {camera_info['Model']}")
    else:
        print("  ⚠️  Camera model not available")
    
    picam2.close()
except Exception as e:
    print(f"  ❌ Camera not available: {e}")
    print("\n  Troubleshooting:")
    print("  1. Check camera is connected properly")
    print("  2. Enable camera interface:")
    print("     sudo raspi-config → Interface Options → Camera → Enable")
    print("  3. Reboot after enabling camera")
    print("  4. Check camera with: libcamera-hello")
    sys.exit(1)

# Step 4: Initialize camera with configuration
print("\n[4/5] Initializing camera...")
picam2 = None
try:
    picam2 = Picamera2()
    
    # Create preview configuration (matching production config)
    config = picam2.create_preview_configuration(
        main={
            "size": (640, 480),
            "format": "RGB888"
        },
        controls={
            "FrameRate": 30.0
        }
    )
    
    print("  Configuring camera (640x480 @ 30fps)...")
    picam2.configure(config)
    
    print("  Starting camera...")
    picam2.start()
    
    # Wait for camera to stabilize
    print("  Waiting for camera to stabilize (2 seconds)...")
    time.sleep(2)
    
    print("  ✓ Camera initialized successfully")
    
except Exception as e:
    print(f"  ❌ Failed to initialize camera: {e}")
    if picam2:
        try:
            picam2.close()
        except:
            pass
    sys.exit(1)

# Step 5: Test frame capture
print("\n[5/5] Testing frame capture...")
try:
    print("  Capturing frame...")
    frame = picam2.capture_array()
    
    if frame is None:
        print("  ❌ Frame capture returned None")
        picam2.stop()
        picam2.close()
        sys.exit(1)
    
    # Check frame properties
    print(f"  ✓ Frame captured successfully!")
    print(f"     Shape: {frame.shape}")
    print(f"     Dtype: {frame.dtype}")
    print(f"     Min value: {frame.min()}")
    print(f"     Max value: {frame.max()}")
    print(f"     Mean value: {frame.mean():.2f}")
    
    # Capture a few more frames to test consistency
    print("\n  Testing multiple frame captures...")
    frame_times = []
    for i in range(5):
        start = time.time()
        test_frame = picam2.capture_array()
        elapsed = time.time() - start
        frame_times.append(elapsed)
        if test_frame is None:
            print(f"  ❌ Frame {i+1} capture failed")
            break
        print(f"     Frame {i+1}: {elapsed*1000:.2f}ms", end="")
    
    if len(frame_times) == 5:
        avg_time = sum(frame_times) / len(frame_times)
        fps_estimate = 1.0 / avg_time if avg_time > 0 else 0
        print(f"\n  ✓ Average capture time: {avg_time*1000:.2f}ms (~{fps_estimate:.1f} fps)")
    
except Exception as e:
    print(f"  ❌ Frame capture failed: {e}")
    picam2.stop()
    picam2.close()
    sys.exit(1)

# Cleanup
print("\n" + "=" * 60)
print("Cleaning up...")
try:
    picam2.stop()
    picam2.close()
    print("✓ Camera closed successfully")
except Exception as e:
    print(f"⚠️  Error during cleanup: {e}")

print("\n" + "=" * 60)
print("✓ Camera test PASSED!")
print("=" * 60)
print("\nCamera is working correctly and ready for use.")
print("You can now use the camera in your drone client application.")
sys.exit(0)

