import cv2
import numpy as np
import time
from picamera2 import Picamera2

# ----------------------------
# CONFIG
# ----------------------------
ONNX_MODEL = "yolo11n.onnx"
INPUT_WIDTH = 320
INPUT_HEIGHT = 320
TARGET_CLASSES = [0, 2]  # person, car
CONFIDENCE_THRESHOLD = 0.5
NMS_THRESHOLD = 0.4

# Class names (only for display)
CLASS_NAMES = {0: "person", 2: "car"}

# ----------------------------
# YOLOv8 Post-processing
# ----------------------------
def decode_yolov8(outputs, img_shape, input_shape):
    """
    Decode YOLOv8 output (1x84x8400) to boxes, scores, classes.
    """
    outputs = outputs[0]  # [1, 84, 8400] > [84, 8400]
    num_candidates = outputs.shape[1]  # 8400
    boxes = []
    scores = []
    class_ids = []

    x_factor = img_shape[1] / input_shape[0]
    y_factor = img_shape[0] / input_shape[1]

    for i in range(num_candidates):
        class_scores = outputs[4:, i]
        max_score = np.max(class_scores)
        if max_score >= CONFIDENCE_THRESHOLD:
            class_id = np.argmax(class_scores)
            if class_id not in TARGET_CLASSES:
                continue

            # Bounding box (center x, center y, width, height)
            cx, cy, w, h = outputs[0:4, i]
            left = int((cx - w / 2) * x_factor)
            top = int((cy - h / 2) * y_factor)
            width = int(w * x_factor)
            height = int(h * y_factor)

            boxes.append([left, top, width, height])
            scores.append(float(max_score))
            class_ids.append(class_id)

    # Apply NMS
    indices = cv2.dnn.NMSBoxes(boxes, scores, CONFIDENCE_THRESHOLD, NMS_THRESHOLD)
    if len(indices) == 0:
        return [], [], []

    if isinstance(indices, list):
        indices = np.array(indices)

    final_boxes = [boxes[i] for i in indices.flatten()]
    final_scores = [scores[i] for i in indices.flatten()]
    final_classes = [class_ids[i] for i in indices.flatten()]

    return final_boxes, final_scores, final_classes

# ----------------------------
# Main
# ----------------------------
def main():
    # Load ONNX model
    net = cv2.dnn.readNetFromONNX(ONNX_MODEL)
    # Use optimized CPU backend (default is fine, but ensure no OpenCL issues)
    net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
    net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

    # Initialize camera
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(
        main={"size": (640, 480), "format": "RGB888"},
        controls={"FrameRate": 30}
    )
    picam2.configure(config)
    picam2.start()

    print("Starting YOLOv8 + OpenCV DNN on Pi 5. Press 'q' to quit.")
    time.sleep(2)  # Let camera warm up

    frame_count = 0
    start_time = time.time()

    while True:
        frame = picam2.capture_array()  # RGB array
        original_frame = frame.copy()

        # Preprocess: resize, normalize, swap channels
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

        # Draw results
        for box, score, cls_id in zip(boxes, scores, class_ids):
            x, y, w, h = box
            label = f"{CLASS_NAMES[cls_id]} {score:.2f}"
            cv2.rectangle(original_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(original_frame, label, (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
       # FPS
        frame_count += 1
        elapsed = time.time() - start_time
        fps = frame_count / elapsed if elapsed > 0 else 0
        cv2.putText(original_frame, f"FPS: {fps:.1f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(original_frame, f"Detections: {len(boxes)}", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # Show
        cv2.imshow("YOLOv8 ONNX - Pi 5", original_frame)

        if cv2.waitKey(1) == ord('q'):
            break

    # Cleanup
    picam2.stop()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
