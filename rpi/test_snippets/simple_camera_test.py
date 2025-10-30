#!/usr/bin/env python3
"""
Simple camera test - minimal test to verify camera works
Based on basic_detection_1.py pattern
"""

import sys
import time

# Import checks with helpful error messages
try:
    import numpy as np
except ImportError:
    print("❌ numpy not found. Install with: pip install numpy")
    sys.exit(1)

try:
    from picamera2 import Picamera2
except ImportError as e:
    print("❌ picamera2 import failed!")
    print(f"   Error: {e}")
    print("\n   Fix:")
    print("   1. Install system packages:")
    print("      sudo apt install -y python3-picamera2 libcamera-dev libcap-dev")
    print("   2. Install Python package:")
    print("      pip install picamera2")
    print("   3. Enable camera: sudo raspi-config → Interface Options → Camera")
    print("   4. Reboot after enabling camera")
    sys.exit(1)

print("=" * 50)
print("Simple Camera Test")
print("=" * 50)

picam2 = None

try:
    print("\n[1/3] Checking camera detection...")
    picam2 = Picamera2()
    
    # Try to access camera properties to verify camera is detected
    try:
        camera_props = picam2.camera_properties
        print("  ✓ Camera detected")
        
        # Try to get camera model if available
        try:
            if 'Model' in camera_props:
                print(f"  ✓ Camera Model: {camera_props['Model']}")
        except:
            pass
    except Exception as e:
        print(f"  ⚠️  Could not read camera properties: {e}")
        print("  (This might be OK, continuing...)")
    
    print("\n[2/3] Configuring camera...")
    config = picam2.create_preview_configuration(
        main={"size": (640, 480), "format": "RGB888"},
        controls={"FrameRate": 30.0}
    )
    picam2.configure(config)
    
    print("\n[3/3] Starting camera...")
    picam2.start()
    
    print("  ✓ Camera started")
    print("  Waiting 2 seconds for stabilization...")
    time.sleep(2)
    
    print("\nCapturing test frames...")
    for i in range(3):
        try:
            frame = picam2.capture_array()
            if frame is not None:
                print(f"  ✓ Frame {i+1}: {frame.shape}, dtype={frame.dtype}")
            else:
                print(f"  ❌ Frame {i+1}: Returned None")
                break
        except Exception as e:
            print(f"  ❌ Frame {i+1}: Capture failed - {e}")
            break
    
    print("\n✓ Camera test successful!")
    
except IndexError as e:
    print(f"\n❌ Camera enumeration failed (list index out of range)")
    print("  This usually means no camera was detected.")
    print("\nTroubleshooting:")
    print("1. Check camera is physically connected")
    print("2. Enable camera interface:")
    print("   sudo raspi-config → Interface Options → Camera → Enable")
    print("3. Reboot after enabling: sudo reboot")
    print("4. Test with system tool:")
    print("   libcamera-hello")
    print("   If this fails, camera hardware may not be detected")
    sys.exit(1)
    
except Exception as e:
    print(f"\n❌ Camera test failed: {e}")
    print(f"   Error type: {type(e).__name__}")
    print("\nTroubleshooting:")
    print("1. Check camera is connected")
    print("2. Enable camera: sudo raspi-config → Interface Options → Camera")
    print("3. Reboot after enabling")
    print("4. Test with: libcamera-hello")
    print("5. Check if camera is in use: sudo lsof /dev/video*")
    sys.exit(1)
    
finally:
    if picam2:
        try:
            picam2.stop()
            picam2.close()
            print("\nCamera closed.")
        except:
            pass

