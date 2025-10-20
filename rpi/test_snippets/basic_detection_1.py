import cv2
from picamera2 import Picamera2
from ultralytics import YOLO
import time

# Load YOLO11n model (nano version for speed)
model = YOLO("yolo11n.pt")

# Initialize camera
picam2 = Picamera2()
config = picam2.create_preview_configuration(
    main={"size": (640, 480), "format": "RGB888"},
    controls={"FrameRate": 20.0}
)
picam2.configure(config)
picam2.start()

# Define target classes: person=0, car=2
TARGET_CLASSES = [0, 2]
CLASS_NAMES = {0: "person", 2: "car"}

print("Starting vision pipeline. Press 'q' to quit.")

frame_count = 0
start_time = time.time()

while True:
    # Capture frame
    frame = picam2.capture_array()
    
    # Run inference
    results = model(frame, verbose=False, imgsz=640, conf=0.5)
    
    # Process results
    annotated_frame = frame.copy()
    detections = []
    
    for r in results:
        boxes = r.boxes
        for box in boxes:
            cls = int(box.cls[0])
            conf = float(box.conf[0])
            if cls in TARGET_CLASSES:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                label = f"{CLASS_NAMES[cls]} {conf:.2f}"
                cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(annotated_frame, label, (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                detections.append((cls, conf, (x1, y1, x2, y2)))
    
    # Calculate FPS
    frame_count += 1
    elapsed = time.time() - start_time
    fps = frame_count / elapsed if elapsed > 0 else 0
    
    # Display FPS and detection count
    cv2.putText(annotated_frame, f"FPS: {fps:.1f}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv2.putText(annotated_frame, f"Detections: {len(detections)}", (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    
    # Show frame
    cv2.imshow("YOLO Detection", annotated_frame)
    
    # Exit on 'q'
    if cv2.waitKey(1) == ord('q'):
        break

# Cleanup
picam2.stop()
cv2.destroyAllWindows()
