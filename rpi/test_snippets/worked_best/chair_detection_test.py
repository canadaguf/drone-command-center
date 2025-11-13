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
    try:
        # Safety check: ensure outputs is valid
        if outputs is None or len(outputs) == 0:
            return [], [], []
        
        outputs = outputs[0]  # [1, 84, 8400] > [84, 8400]
        
        # Safety check: ensure output shape is valid
        if len(outputs.shape) != 2:
            print(f"  Warning: Unexpected output shape: {outputs.shape}")
            return [], [], []
        
        num_candidates = outputs.shape[1]  # 8400
        num_classes = outputs.shape[0] - 4  # 80 classes (84 - 4)
        
        if num_candidates == 0:
            return [], [], []
        
        boxes = []
        scores = []
        class_ids = []

        # Calculate scaling factors
        x_factor = img_shape[1] / input_shape[0]
        y_factor = img_shape[0] / input_shape[1]

        # Process each candidate with bounds checking
        for i in range(num_candidates):
            try:
                # Safety check: ensure we can access the array
                if i >= outputs.shape[1]:
                    break
                
                class_scores = outputs[4:, i]
                if len(class_scores) == 0:
                    continue
                
                max_score = np.max(class_scores)
                
                if max_score >= CONFIDENCE_THRESHOLD:
                    class_id = np.argmax(class_scores)
                    
                    # Only process target classes (chairs)
                    if class_id not in TARGET_CLASSES:
                        continue

                    # Extract bounding box (center x, center y, width, height)
                    cx, cy, w, h = outputs[0:4, i]
                    
                    # Validate bounding box values
                    if not (np.isfinite(cx) and np.isfinite(cy) and np.isfinite(w) and np.isfinite(h)):
                        continue
                    
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
            except (IndexError, ValueError) as e:
                # Skip this candidate if there's an error
                continue
    except Exception as e:
        print(f"  Error in decode_yolov8: {e}")
        import traceback
        traceback.print_exc()
        return [], [], []

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
    print("  Running in HEADLESS mode (no display)")
    print("  Press Ctrl+C to stop")
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
            
            # Run inference with error handling
            try:
                net.setInput(blob)
                outputs = net.forward()
                
                # Safety check: ensure outputs is valid
                if outputs is None:
                    print(f"[{frame_count}] Warning: Model returned None output")
                    continue
                
                # Debug: print output shape on first frame
                if frame_count == 1:
                    print(f"[{frame_count}] Model output shape: {outputs.shape if hasattr(outputs, 'shape') else type(outputs)}")
                    if hasattr(outputs, 'shape') and len(outputs.shape) > 0:
                        print(f"  Output type: {type(outputs)}")
                        if len(outputs) > 0:
                            print(f"  First element shape: {outputs[0].shape if hasattr(outputs[0], 'shape') else 'N/A'}")
                
                # Post-process
                boxes, scores, class_ids = decode_yolov8(
                    outputs,
                    img_shape=frame.shape[:2],
                    input_shape=(INPUT_HEIGHT, INPUT_WIDTH)
                )
            except Exception as e:
                print(f"[{frame_count}] Error during inference: {e}")
                import traceback
                traceback.print_exc()
                continue
            except SystemError as e:
                # Catch segfault-related errors
                print(f"[{frame_count}] System error (possible segfault): {e}")
                print("  This might indicate a memory or model compatibility issue")
                break
            
            # Print detection results to console (headless mode)
            if len(boxes) > 0:
                detection_count += len(boxes)
                print(f"\n[{frame_count}] ✓ Detected {len(boxes)} chair(s) (FPS: {fps:.1f}):")
                
                for i, (box, score, cls_id) in enumerate(zip(boxes, scores, class_ids)):
                    x, y, w, h = box
                    center_x = x + w // 2
                    center_y = y + h // 2
                    
                    print(f"  Chair {i+1}:")
                    print(f"    Confidence: {score:.3f}")
                    print(f"    BBox: ({x}, {y}, {x+w}, {y+h})")
                    print(f"    Center: ({center_x}, {center_y})")
                    print(f"    Size: {w}x{h} pixels")
                    print(f"    Height ratio: {h/480:.3f} (for distance estimation)")
            
            # Calculate FPS
            frame_count += 1
            elapsed = time.time() - start_time
            fps = frame_count / elapsed if elapsed > 0 else 0
            
            # Headless mode - no display needed
            # Just print detection info to console
            if len(boxes) == 0 and frame_count % 30 == 0:
                # Print status every 30 frames when no detections
                print(f"[{frame_count}] No chairs detected (FPS: {fps:.1f})")
            
            # Small delay to avoid overwhelming the system and reduce segfault risk
            time.sleep(0.01)  # 10ms delay = ~100Hz max
            
            # Exit check (non-blocking, for headless mode)
            # Note: In headless mode, we can't detect 'q' keypress
            # Use Ctrl+C to stop instead
            
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

