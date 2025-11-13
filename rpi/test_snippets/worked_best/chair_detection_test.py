"""
Test script for chair detection using YOLO11n ONNX model.
Tests chair detection (COCO class_id 56) to verify detection before starting client.
"""

import cv2
import numpy as np
import time
import os
import sys
from picamera2 import Picamera2

# ----------------------------
# CONFIG
# ----------------------------
# Try multiple possible model paths
POSSIBLE_MODEL_PATHS = [
    "/home/ilya/models/yolo11n.onnx",
    "/home/pi/models/yolo11n.onnx",
    "yolo11n.onnx",  # Current directory
    os.path.join(os.path.dirname(__file__), "..", "..", "models", "yolo11n.onnx")
]

INPUT_WIDTH = 320
INPUT_HEIGHT = 320
TARGET_CLASSES = [56]  # COCO class_id 56 = chair
CONFIDENCE_THRESHOLD = 0.5
NMS_THRESHOLD = 0.4

# Class names (COCO dataset)
CLASS_NAMES = {56: "chair"}

# ----------------------------
# YOLO11 Post-processing (matches YOLODetector class)
# ----------------------------
def decode_yolov8(outputs, img_shape, input_shape):
    """
    Decode YOLO11 output (1x84x8400) to boxes, scores, classes.
    Matches the post-processing in YOLODetector class.
    """
    outputs = outputs[0]  # [1, 84, 8400] > [84, 8400]
    num_candidates = outputs.shape[1]  # 8400
    boxes = []
    scores = []
    class_ids = []

    # Calculate scaling factors
    x_factor = img_shape[1] / input_shape[0]
    y_factor = img_shape[0] / input_shape[1]

    # Process each candidate
    for i in range(num_candidates):
        class_scores = outputs[4:, i]
        max_score = np.max(class_scores)
        
        if max_score >= CONFIDENCE_THRESHOLD:
            class_id = np.argmax(class_scores)
            
            # Only process target classes (chairs)
            if class_id not in TARGET_CLASSES:
                continue

            # Extract bounding box (center x, center y, width, height)
            cx, cy, w, h = outputs[0:4, i]
            
            # Convert to image coordinates
            left = int((cx - w / 2) * x_factor)
            top = int((cy - h / 2) * y_factor)
            width = int(w * x_factor)
            height = int(h * y_factor)
            
            # Ensure coordinates are within image bounds
            left = max(0, min(left, img_shape[1] - 1))
            top = max(0, min(top, img_shape[0] - 1))
            width = max(1, min(width, img_shape[1] - left))
            height = max(1, min(height, img_shape[0] - top))

            boxes.append([left, top, width, height])
            scores.append(float(max_score))
            class_ids.append(class_id)

    # Apply NMS
    if len(boxes) > 0:
        indices = cv2.dnn.NMSBoxes(
            boxes, scores, CONFIDENCE_THRESHOLD, NMS_THRESHOLD
        )
        
        if len(indices) > 0:
            if isinstance(indices, list):
                indices = np.array(indices)
            
            final_boxes = [boxes[i] for i in indices.flatten()]
            final_scores = [scores[i] for i in indices.flatten()]
            final_classes = [class_ids[i] for i in indices.flatten()]
            
            return final_boxes, final_scores, final_classes
    
    return [], [], []

# ----------------------------
# Main
# ----------------------------
def main():
    print("=" * 60)
    print("Chair Detection Test")
    print("=" * 60)
    
    # Find model file
    print("\n[1/4] Looking for YOLO model...")
    model_path = None
    for path in POSSIBLE_MODEL_PATHS:
        if os.path.exists(path):
            model_path = path
            print(f"  ✓ Found model: {path}")
            break
    
    if model_path is None:
        print("  ❌ Model not found in any of these locations:")
        for path in POSSIBLE_MODEL_PATHS:
            print(f"     - {path}")
        print("\n  Please ensure yolo11n.onnx is in one of these locations.")
        sys.exit(1)
    
    # Load ONNX model
    print("\n[2/4] Loading YOLO model...")
    try:
        net = cv2.dnn.readNetFromONNX(model_path)
        net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
        print("  ✓ Model loaded successfully")
    except Exception as e:
        print(f"  ❌ Failed to load model: {e}")
        sys.exit(1)
    
    # Initialize camera
    print("\n[3/4] Initializing camera...")
    try:
        picam2 = Picamera2()
        config = picam2.create_preview_configuration(
            main={"size": (640, 480), "format": "RGB888"},
            controls={"FrameRate": 30}
        )
        picam2.configure(config)
        picam2.start()
        print("  ✓ Camera initialized")
        print("  Waiting 2 seconds for camera to stabilize...")
        time.sleep(2)
    except Exception as e:
        print(f"  ❌ Failed to initialize camera: {e}")
        sys.exit(1)
    
    # Detection loop
    print("\n[4/4] Starting detection loop...")
    print("  Target class: chair (COCO class_id 56)")
    print("  Confidence threshold: {:.2f}".format(CONFIDENCE_THRESHOLD))
    print("  Press 'q' to quit")
    print("=" * 60)
    
    frame_count = 0
    detection_count = 0
    start_time = time.time()
    
    try:
        while True:
            # Capture frame
            frame = picam2.capture_array()  # RGB array
            original_frame = frame.copy()
            
            # Preprocess: resize, normalize, swap channels (matches YOLODetector)
            blob = cv2.dnn.blobFromImage(
                frame,
                scalefactor=1/255.0,
                size=(INPUT_WIDTH, INPUT_HEIGHT),
                mean=(0, 0, 0),
                swapRB=True,
                crop=False
            )
            
            # Run inference
            net.setInput(blob)
            outputs = net.forward()
            
            # Post-process
            boxes, scores, class_ids = decode_yolov8(
                outputs,
                img_shape=frame.shape[:2],
                input_shape=(INPUT_HEIGHT, INPUT_WIDTH)
            )
            
            # Draw results and print to console
            if len(boxes) > 0:
                detection_count += len(boxes)
                print(f"\n[{frame_count}] Detected {len(boxes)} chair(s):")
                
                for i, (box, score, cls_id) in enumerate(zip(boxes, scores, class_ids)):
                    x, y, w, h = box
                    center_x = x + w // 2
                    center_y = y + h // 2
                    
                    print(f"  Chair {i+1}:")
                    print(f"    Confidence: {score:.3f}")
                    print(f"    BBox: ({x}, {y}, {x+w}, {y+h})")
                    print(f"    Center: ({center_x}, {center_y})")
                    print(f"    Size: {w}x{h} pixels")
                    
                    # Draw bounding box
                    label = f"{CLASS_NAMES[cls_id]} {score:.2f}"
                    cv2.rectangle(original_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.putText(original_frame, label, (x, y - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    
                    # Draw center point
                    cv2.circle(original_frame, (center_x, center_y), 5, (0, 0, 255), -1)
            
            # Calculate FPS
            frame_count += 1
            elapsed = time.time() - start_time
            fps = frame_count / elapsed if elapsed > 0 else 0
            
            # Draw FPS and detection count
            info_text = f"FPS: {fps:.1f} | Chairs: {len(boxes)} | Total: {detection_count}"
            cv2.putText(original_frame, info_text, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # Draw target class info
            cv2.putText(original_frame, "Detecting: CHAIR (class 56)", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            
            # Show frame
            cv2.imshow("Chair Detection Test", original_frame)
            
            # Exit on 'q'
            if cv2.waitKey(1) == ord('q'):
                break
            
            # Print summary every 30 frames
            if frame_count % 30 == 0:
                avg_fps = frame_count / elapsed if elapsed > 0 else 0
                print(f"\n--- Summary (after {frame_count} frames) ---")
                print(f"  Average FPS: {avg_fps:.1f}")
                print(f"  Total detections: {detection_count}")
                print(f"  Detection rate: {detection_count/frame_count*100:.1f}%")
                print("-" * 60)
    
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    except Exception as e:
        print(f"\n\nError in detection loop: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Cleanup
        print("\nCleaning up...")
        try:
            picam2.stop()
            cv2.destroyAllWindows()
            print("✓ Camera stopped")
        except:
            pass
        
        # Final summary
        elapsed = time.time() - start_time
        if frame_count > 0:
            avg_fps = frame_count / elapsed if elapsed > 0 else 0
            print("\n" + "=" * 60)
            print("Final Summary:")
            print(f"  Total frames: {frame_count}")
            print(f"  Total detections: {detection_count}")
            print(f"  Average FPS: {avg_fps:.1f}")
            print(f"  Detection rate: {detection_count/frame_count*100:.1f}%")
            print("=" * 60)
        
        if detection_count == 0:
            print("\n⚠️  WARNING: No chairs detected!")
            print("  Possible issues:")
            print("  1. No chairs in view")
            print("  2. Confidence threshold too high (current: {:.2f})".format(CONFIDENCE_THRESHOLD))
            print("  3. Lighting conditions")
            print("  4. Model path incorrect")
        else:
            print("\n✓ Chair detection is working!")

if __name__ == "__main__":
    main()

