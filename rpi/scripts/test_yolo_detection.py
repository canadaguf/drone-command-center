#!/usr/bin/env python3
"""
YOLO Detection Test Script
Tests YOLO person detection with camera and displays results.
Similar to test_hardware.py but focused on vision pipeline.
"""

import sys
import time
import logging
import cv2
import numpy as np
from pathlib import Path

# Add drone_client to path
sys.path.append(str(Path(__file__).parent.parent))

from drone_client.config import Config
from drone_client.sensors.camera import CameraManager
from drone_client.vision.yolo_detector import YOLODetector
from drone_client.vision.person_tracker import PersonTracker

# Setup logging
logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')
logger = logging.getLogger(__name__)


def test_yolo_detection_with_camera():
    """Test YOLO detection with live camera feed."""
    print("\n" + "=" * 60)
    print("YOLO Detection Test with Camera")
    print("=" * 60)
    
    # Initialize config
    try:
        config = Config()
        vision_config = config.get_vision_config()
        camera_config = config.get_camera_config()
    except Exception as e:
        print(f"❌ Failed to load config: {e}")
        return False
    
    # Check model path
    model_path = vision_config.get('model_path', '/home/pi/models/yolo11n.onnx')
    if not Path(model_path).exists():
        print(f"❌ YOLO model not found: {model_path}")
        print(f"   Please run: python3 {Path(__file__).parent}/download_yolo_model.py")
        return False
    
    print(f"✓ Model path: {model_path}")
    
    # Initialize YOLO detector
    try:
        yolo = YOLODetector(
            model_path=model_path,
            input_size=vision_config.get('input_size', 320),
            confidence_threshold=vision_config.get('confidence', 0.5)
        )
        
        if yolo.net is None:
            print("❌ YOLO model failed to load")
            return False
        
        print("✓ YOLO detector initialized")
    except Exception as e:
        print(f"❌ Failed to initialize YOLO detector: {e}")
        return False
    
    # Initialize camera
    try:
        camera = CameraManager(camera_config)
        if not camera.initialize():
            print("❌ Camera initialization failed")
            return False
        print("✓ Camera initialized")
        
        # Wait for camera to stabilize
        print("  Warming up camera...")
        time.sleep(2)
        
        # Test frame capture
        test_frame = camera.capture_frame()
        if test_frame is None:
            print("❌ Test frame capture failed")
            camera.cleanup()
            return False
        print(f"✓ Camera working: {test_frame.shape}")
    except Exception as e:
        print(f"❌ Camera error: {e}")
        return False
    
    # Initialize person tracker
    try:
        tracker = PersonTracker()
        print("✓ Person tracker initialized")
    except Exception as e:
        print(f"❌ Failed to initialize tracker: {e}")
        camera.cleanup()
        return False
    
    # Detection loop
    print("\n" + "-" * 60)
    print("Starting detection loop...")
    print("Press 'q' to quit, 's' to save current frame")
    print("-" * 60)
    
    frame_count = 0
    detection_count = 0
    start_time = time.time()
    
    try:
        while True:
            # Capture frame
            frame = camera.capture_frame()
            if frame is None:
                print("⚠️  Frame capture returned None, skipping...")
                time.sleep(0.1)
                continue
            
            # Run YOLO detection
            detections = yolo.detect(frame)
            
            # Update tracker
            tracked_persons = tracker.update(detections)
            
            # Draw detections
            annotated_frame = frame.copy()
            
            # Draw all raw detections
            for det in detections:
                x1, y1, x2, y2 = det['bbox']
                conf = det['confidence']
                class_name = det['class_name']
                
                # Draw bounding box
                cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                
                # Draw label
                label = f"{class_name} {conf:.2f}"
                label_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)[0]
                cv2.rectangle(annotated_frame, (x1, y1 - label_size[1] - 10), 
                             (x1 + label_size[0], y1), (0, 255, 0), -1)
                cv2.putText(annotated_frame, label, (x1, y1 - 5),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
            
            # Draw tracked persons with IDs
            for person in tracked_persons:
                x1, y1, x2, y2 = person['bbox']
                person_id = person['id']
                conf = person['confidence']
                
                # Draw thicker box for tracked persons
                cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), (255, 0, 0), 3)
                
                # Draw ID label
                id_label = f"ID:{person_id} {conf:.2f}"
                id_label_size = cv2.getTextSize(id_label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)[0]
                cv2.rectangle(annotated_frame, (x1, y2), 
                             (x1 + id_label_size[0] + 10, y2 + id_label_size[1] + 10), 
                             (255, 0, 0), -1)
                cv2.putText(annotated_frame, id_label, (x1 + 5, y2 + id_label_size[1] + 5),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            # Calculate FPS
            frame_count += 1
            elapsed = time.time() - start_time
            fps = frame_count / elapsed if elapsed > 0 else 0
            
            # Draw stats
            stats_text = [
                f"FPS: {fps:.1f}",
                f"Detections: {len(detections)}",
                f"Tracked: {len(tracked_persons)}",
                f"Frame: {frame_count}"
            ]
            
            y_offset = 30
            for i, text in enumerate(stats_text):
                cv2.putText(annotated_frame, text, (10, y_offset + i * 25),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # Show frame
            cv2.imshow("YOLO Detection Test", annotated_frame)
            
            # Print detection info to console (every 30 frames)
            if frame_count % 30 == 0:
                if len(tracked_persons) > 0:
                    print(f"\n[{frame_count}] Detected {len(tracked_persons)} person(s):")
                    for person in tracked_persons:
                        print(f"  - Person ID {person['id']}: confidence={person['confidence']:.2f}, "
                              f"bbox={person['bbox']}, center={person['center']}")
                    detection_count += len(tracked_persons)
                else:
                    print(f"[{frame_count}] No detections (FPS: {fps:.1f})")
            
            # Handle keyboard input
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                print("\n✓ Quitting...")
                break
            elif key == ord('s'):
                # Save current frame
                timestamp = int(time.time())
                filename = f"detection_test_{timestamp}.jpg"
                cv2.imwrite(filename, annotated_frame)
                print(f"✓ Saved frame to {filename}")
            
            # Small delay to prevent overheating
            time.sleep(0.02)  # ~50 Hz max
            
    except KeyboardInterrupt:
        print("\n✓ Interrupted by user")
    except Exception as e:
        print(f"\n❌ Error in detection loop: {e}")
        import traceback
        traceback.print_exc()
        return False
    finally:
        # Cleanup
        cv2.destroyAllWindows()
        camera.cleanup()
        print("✓ Cleanup complete")
    
    # Summary
    elapsed_total = time.time() - start_time
    avg_fps = frame_count / elapsed_total if elapsed_total > 0 else 0
    
    print("\n" + "=" * 60)
    print("Test Summary:")
    print("=" * 60)
    print(f"Total frames: {frame_count}")
    print(f"Average FPS: {avg_fps:.2f}")
    print(f"Total detections: {detection_count}")
    print(f"Detection rate: {detection_count / frame_count * 100:.1f}%" if frame_count > 0 else "N/A")
    print("=" * 60)
    
    if frame_count > 0:
        print("✅ Detection test completed successfully!")
        return True
    else:
        print("❌ No frames captured")
        return False


def test_yolo_only():
    """Quick test of YOLO detector without camera (uses dummy frame)."""
    print("\n" + "=" * 60)
    print("YOLO Detector Quick Test (No Camera)")
    print("=" * 60)
    
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
        
        if yolo.net is None:
            print("❌ YOLO model failed to load")
            return False
        
        print("✓ YOLO detector initialized")
        
        # Test with dummy frame
        dummy_frame = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        detections = yolo.detect(dummy_frame)
        
        print(f"✓ Detection test completed: {len(detections)} detections on dummy frame")
        return True
        
    except Exception as e:
        print(f"❌ Test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def main():
    """Main entry point."""
    print("🔍 YOLO Detection Test Script")
    print("=" * 60)
    
    # Check if we should run with or without camera
    if len(sys.argv) > 1 and sys.argv[1] == '--no-camera':
        # Quick test without camera
        success = test_yolo_only()
    else:
        # Full test with camera
        success = test_yolo_detection_with_camera()
    
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()

