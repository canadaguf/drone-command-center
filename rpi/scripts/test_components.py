#!/usr/bin/env python3
"""
Component test script for drone client.
Tests individual components in isolation.
"""

import sys
import time
import logging
import asyncio
from pathlib import Path

# Add drone_client to path
sys.path.append(str(Path(__file__).parent.parent))

from drone_client.config import Config
from drone_client.controllers.pid_controller import PIDManager
from drone_client.controllers.mavlink_controller import MAVLinkController
from drone_client.vision.person_tracker import PersonTracker
from drone_client.vision.depth_estimator import DepthEstimator
from drone_client.vision.yolo_detector import YOLODetector
from drone_client.sensors.camera import CameraManager
from drone_client.sensors.telemetry import TelemetryCollector
from drone_client.safety.rc_override_monitor import RCOverrideMonitor
from drone_client.safety.lost_target_handler import LostTargetHandler
from drone_client.safety.battery_monitor import BatteryMonitor

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def test_pid_controller():
    """Test PID controller."""
    print("\n=== Testing PID Controller ===")
    try:
        config = {
            'yaw': {'kp': 0.5, 'ki': 0.0, 'kd': 0.1},
            'pitch': {'kp': 0.3, 'ki': 0.0, 'kd': 0.05},
            'roll': {'kp': 0.3, 'ki': 0.0, 'kd': 0.05}
        }
        
        pid_manager = PIDManager(config)
        print("✅ PID manager created")
        
        # Test PID update
        errors = {'yaw': 0.1, 'pitch': 0.2, 'roll': 0.15}
        outputs = pid_manager.update(errors)
        print(f"📊 PID outputs: {outputs}")
        
        # Test reset
        pid_manager.reset_all()
        print("✅ PID controllers reset")
        
        return True
    except Exception as e:
        print(f"❌ PID test failed: {e}")
        return False

def test_person_tracker():
    """Integration-style test: run YOLO on a camera frame (if available) and track persons."""
    print("\n=== Testing Person Tracker (Camera + YOLO) ===")
    try:
        cfg = Config()
        cam_cfg = cfg.get_camera_config()
        vision_cfg = cfg.get_vision_config()

        # Initialize camera
        camera = CameraManager(cam_cfg)
        if not camera.initialize():
            print("⚠️ Camera not available; skipping person tracker integration test")
            return True  # not a failure if hardware is absent

        # Ensure model exists
        model_path = vision_cfg.get('model_path', '/home/ilya/models/yolo11n.onnx')
        import os
        if not os.path.exists(model_path):
            print(f"⚠️ YOLO model not found at {model_path}; skipping detection")
            camera.cleanup()
            return True

        yolo = YOLODetector(
            model_path=model_path,
            input_size=vision_cfg.get('input_size', 320),
            confidence_threshold=vision_cfg.get('confidence', 0.5)
        )

        # Capture a frame and run detection
        frame = camera.capture_frame()
        if frame is None:
            print("❌ Failed to capture frame from camera")
            camera.cleanup()
            return False

        detections = yolo.detect(frame)
        tracker = PersonTracker()
        tracked = tracker.update(detections)
        print(f"📊 Persons detected: {len(detections)}, tracked: {len(tracked)}")

        camera.cleanup()
        return True
    except Exception as e:
        print(f"❌ Person tracker test failed: {e}")
        return False

def test_depth_estimator():
    """Test depth estimator."""
    print("\n=== Testing Depth Estimator ===")
    try:
        config = {
            'width': 640,
            'height': 480,
            'fov_horizontal': 120,
            'fov_vertical': 90
        }
        
        estimator = DepthEstimator(config)
        print("✅ Depth estimator created")
        
        # Test distance estimation
        bbox = (100, 100, 200, 300)
        distance = estimator.estimate_distance(bbox, tof_distance=None)
        print(f"📏 Estimated distance: {distance}m")
        
        # Test with TOF
        distance_with_tof = estimator.estimate_distance(bbox, tof_distance=2.5)
        print(f"📏 Distance with TOF: {distance_with_tof}m")
        
        # Test status
        status = estimator.get_status()
        print(f"📊 Estimator status: {status}")
        
        return True
    except Exception as e:
        print(f"❌ Depth estimator test failed: {e}")
        return False

def test_telemetry_collector():
    """Integration-style test: pull live telemetry from MAVLink if connected."""
    print("\n=== Testing Telemetry Collector (MAVLink) ===")
    try:
        cfg = Config()
        mav_cfg = cfg.get_mavlink_config()
        collector = TelemetryCollector()

        mav = MAVLinkController(mav_cfg)
        if not mav.connect():
            print("⚠️ MAVLink not connected; skipping live telemetry test")
            return True

        # Give MAVLink a moment to stream messages
        time.sleep(1.0)
        telem = mav.get_telemetry()
        collector.update_from_mavlink(telem)
        telemetry = collector.get_telemetry()
        print(f"📊 Live telemetry snapshot: {telemetry}")
        mav.disconnect()
        return True
    except Exception as e:
        print(f"❌ Telemetry collector test failed: {e}")
        return False

def test_rc_override_monitor():
    """Test RC override monitor."""
    print("\n=== Testing RC Override Monitor ===")
    try:
        config = {
            'detection_enabled': True,
            'threshold': 100,
            'neutral_timeout': 2.0
        }
        
        monitor = RCOverrideMonitor(config)
        print("✅ RC override monitor created")
        
        # Test with neutral channels
        neutral_channels = {
            'roll': 1500,
            'pitch': 1500,
            'yaw': 1500,
            'throttle': 1500
        }
        override = monitor.update(neutral_channels)
        print(f"📊 Override status: {override}")
        
        # Test with active channels
        active_channels = {
            'roll': 1600,
            'pitch': 1500,
            'yaw': 1500,
            'throttle': 1500
        }
        override = monitor.update(active_channels)
        print(f"📊 Override status: {override}")
        
        # Test status
        status = monitor.get_status()
        print(f"📊 Monitor status: {status}")
        
        return True
    except Exception as e:
        print(f"❌ RC override monitor test failed: {e}")
        return False

def test_lost_target_handler():
    """Test lost target handler."""
    print("\n=== Testing Lost Target Handler ===")
    try:
        config = {
            'hover_timeout': 2.0,
            'land_timeout': 10.0
        }
        
        handler = LostTargetHandler(config)
        print("✅ Lost target handler created")
        
        # Test with target found
        status = handler.update(True)
        print(f"📊 Status with target: {status.value}")
        
        # Test with target lost
        status = handler.update(False)
        print(f"📊 Status without target: {status.value}")
        
        return True
    except Exception as e:
        print(f"❌ Lost target handler test failed: {e}")
        return False

def test_battery_monitor():
    """Test battery monitor."""
    print("\n=== Testing Battery Monitor ===")
    try:
        config = {
            'warn_percent': 30,
            'critical_percent': 20,
            'auto_land_percent': 15
        }
        
        monitor = BatteryMonitor(config)
        print("✅ Battery monitor created")
        
        # Test with normal battery
        status = monitor.update(80.0, 22.5)
        print(f"📊 Status at 80%: {status.value}")
        
        # Test with low battery
        status = monitor.update(25.0, 21.0)
        print(f"📊 Status at 25%: {status.value}")
        
        # Test with critical battery
        status = monitor.update(15.0, 20.5)
        print(f"📊 Status at 15%: {status.value}")
        
        return True
    except Exception as e:
        print(f"❌ Battery monitor test failed: {e}")
        return False

def main():
    """Run all component tests."""
    print("🧪 Drone Client Component Test Suite")
    print("=" * 50)
    
    tests = [
        ("PID Controller", test_pid_controller),
        ("Person Tracker", test_person_tracker),
        ("Depth Estimator", test_depth_estimator),
        ("Telemetry Collector", test_telemetry_collector),
        ("RC Override Monitor", test_rc_override_monitor),
        ("Lost Target Handler", test_lost_target_handler),
        ("Battery Monitor", test_battery_monitor)
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
        print("🎉 All component tests passed!")
        return 0
    else:
        print("⚠️ Some component tests failed.")
        return 1

if __name__ == "__main__":
    sys.exit(main())
