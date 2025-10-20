#!/usr/bin/env python3
"""
Hardware test script for drone client components.
Tests all hardware interfaces and components.
"""

import sys
import time
import logging
from pathlib import Path

# Add drone_client to path
sys.path.append(str(Path(__file__).parent.parent))

from drone_client.config import Config
from drone_client.controllers.mavlink_controller import MAVLinkController
from drone_client.sensors.camera import CameraManager
from drone_client.sensors.tof_manager import TOFManager
from drone_client.vision.yolo_detector import YOLODetector

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def test_mavlink():
    """Test MAVLink connection."""
    print("\n=== Testing MAVLink Connection ===")
    try:
        config = Config()
        mavlink_config = config.get_mavlink_config()
        
        mavlink = MAVLinkController(mavlink_config)
        if mavlink.connect():
            print("✅ MAVLink connection successful")
            
            # Test telemetry
            telemetry = mavlink.get_telemetry()
            print(f"📊 Telemetry: {telemetry}")
            
            # Test system info
            info = mavlink.get_system_info()
            print(f"🔧 System info: {info}")
            
            mavlink.disconnect()
            return True
        else:
            print("❌ MAVLink connection failed")
            return False
    except Exception as e:
        print(f"❌ MAVLink test failed: {e}")
        return False

def test_camera():
    """Test camera."""
    print("\n=== Testing Camera ===")
    try:
        config = Config()
        camera_config = config.get_camera_config()
        
        camera = CameraManager(camera_config)
        if camera.initialize():
            print("✅ Camera initialization successful")
            
            # Test frame capture
            frame = camera.capture_frame()
            if frame is not None:
                print(f"📷 Frame captured: {frame.shape}")
                camera.cleanup()
                return True
            else:
                print("❌ Frame capture failed")
                return False
        else:
            print("❌ Camera initialization failed")
            return False
    except Exception as e:
        print(f"❌ Camera test failed: {e}")
        return False

def test_tof_sensors():
    """Test TOF sensors."""
    print("\n=== Testing TOF Sensors ===")
    try:
        config = Config()
        tof_config = config.get_tof_config()
        
        tof = TOFManager(tof_config)
        if tof.start_reading():
            print("✅ TOF manager started")
            
            # Wait for readings
            time.sleep(2)
            
            # Get readings
            readings = tof.get_all_readings()
            print(f"📏 TOF readings: {readings}")
            
            # Test specific sensors
            forward_dist = tof.get_forward_distance()
            down_dist = tof.get_down_distance()
            print(f"📏 Forward: {forward_dist}m, Down: {down_dist}m")
            
            tof.stop_reading()
            return True
        else:
            print("❌ TOF manager failed to start")
            return False
    except Exception as e:
        print(f"❌ TOF test failed: {e}")
        return False

def test_yolo():
    """Test YOLO detector."""
    print("\n=== Testing YOLO Detector ===")
    try:
        config = Config()
        vision_config = config.get_vision_config()
        
        model_path = vision_config.get('model_path', '/home/pi/models/yolo11n.onnx')
        if not Path(model_path).exists():
            print(f"❌ YOLO model not found: {model_path}")
            return False
        
        yolo = YOLODetector(
            model_path=model_path,
            input_size=vision_config.get('input_size', 320),
            confidence_threshold=vision_config.get('confidence', 0.5)
        )
        
        print("✅ YOLO detector initialized")
        
        # Test with dummy frame
        import numpy as np
        dummy_frame = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        detections = yolo.detect(dummy_frame)
        print(f"🔍 Detections: {len(detections)}")
        
        return True
    except Exception as e:
        print(f"❌ YOLO test failed: {e}")
        return False

def test_i2c():
    """Test I2C interface."""
    print("\n=== Testing I2C Interface ===")
    try:
        import smbus2
        
        # Test I2C bus
        bus = smbus2.SMBus(1)
        print("✅ I2C bus accessible")
        
        # Test TCA9548A multiplexer
        multiplexer_addr = 0x70
        try:
            bus.read_byte(multiplexer_addr)
            print("✅ TCA9548A multiplexer found")
        except:
            print("⚠️ TCA9548A multiplexer not found (may be normal)")
        
        bus.close()
        return True
    except Exception as e:
        print(f"❌ I2C test failed: {e}")
        return False

def test_gpio():
    """Test GPIO interface."""
    print("\n=== Testing GPIO Interface ===")
    try:
        import RPi.GPIO as GPIO
        
        # Test GPIO setup
        GPIO.setmode(GPIO.BCM)
        print("✅ GPIO interface accessible")
        
        GPIO.cleanup()
        return True
    except Exception as e:
        print(f"❌ GPIO test failed: {e}")
        return False

def main():
    """Run all hardware tests."""
    print("🔧 Drone Client Hardware Test Suite")
    print("=" * 50)
    
    tests = [
        ("I2C Interface", test_i2c),
        ("GPIO Interface", test_gpio),
        ("MAVLink Connection", test_mavlink),
        ("Camera", test_camera),
        ("TOF Sensors", test_tof_sensors),
        ("YOLO Detector", test_yolo)
    ]
    
    results = []
    
    for test_name, test_func in tests:
        try:
            result = test_func()
            results.append((test_name, result))
        except Exception as e:
            print(f"❌ {test_name} test crashed: {e}")
            results.append((test_name, False))
    
    # Summary
    print("\n" + "=" * 50)
    print("📊 Test Results Summary:")
    print("=" * 50)
    
    passed = 0
    total = len(results)
    
    for test_name, result in results:
        status = "✅ PASS" if result else "❌ FAIL"
        print(f"{status} {test_name}")
        if result:
            passed += 1
    
    print(f"\n📈 Results: {passed}/{total} tests passed")
    
    if passed == total:
        print("🎉 All tests passed! Hardware is ready.")
        return 0
    else:
        print("⚠️ Some tests failed. Check hardware connections.")
        return 1

if __name__ == "__main__":
    sys.exit(main())
