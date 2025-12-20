#!/usr/bin/env python3

import os
import time
import cv2
import numpy as np
import datetime
from picamera2 import Picamera2
from picamera2.encoders import H264Encoder
from picamera2.outputs import FfmpegOutput
from picamera2.array import MappedArray
import onnxruntime as ort

# --- Configuration ---
MODEL_PATH = "/home/ilya/models/yolo11n.onnx"
VIDEO_DIR = "/home/ilya/videos"
os.makedirs(VIDEO_DIR, exist_ok=True)

# YOLO settings
INPUT_SIZE = (320, 320)  # (width, height)
PERSON_CLASS_ID = 0
CONF_THRESHOLD = 0.5
IOU_THRESHOLD = 0.4

# Initialize ONNX model
ort_session = ort.InferenceSession(MODEL_PATH, providers=['CPUExecutionProvider'])
print(f"Loaded YOLO model from {MODEL_PATH}")

# Simple tracker (frame-to-frame ID assignment based on IoU)
class SimpleTracker:
    def __init__(self, iou_threshold=0.4):
        self.next_id = 1
        self.last_boxes = []  # [(x1, y1, x2, y2), ...]
        self.last_ids = []
        self.iou_threshold = iou_threshold

    def update(self, current_boxes):
        if not self.last_boxes or not current_boxes:
            # Reset IDs if gap or first frame
            new_ids = list(range(self.next_id, self.next_id + len(current_boxes)))
            self.next_id += len(current_boxes)
            self.last_boxes = current_boxes
            self.last_ids = new_ids
            return new_ids

        # Compute IoU matrix
        iou_matrix = np.zeros((len(self.last_boxes), len(current_boxes)))
        for i, box1 in enumerate(self.last_boxes):
            for j, box2 in enumerate(current_boxes):
                iou_matrix[i, j] = self._box_iou(box1, box2)

        # Greedy matching
        assigned_ids = [None] * len(current_boxes)
        used_last = set()
        used_curr = set()

        # Sort by IoU descending
        matches = []
        for i in range(len(self.last_boxes)):
            for j in range(len(current_boxes)):
                matches.append((iou_matrix[i, j], i, j))
        matches.sort(reverse=True, key=lambda x: x[0])

        for iou_val, i, j in matches:
            if iou_val < self.iou_threshold:
                break
            if i in used_last or j in used_curr:
                continue
            assigned_ids[j] = self.last_ids[i]
            used_last.add(i)
            used_curr.add(j)

        # Assign new IDs to unmatched detections
        for j in range(len(current_boxes)):
            if assigned_ids[j] is None:
                assigned_ids[j] = self.next_id
                self.next_id += 1

        self.last_boxes = current_boxes
        self.last_ids = assigned_ids
        return assigned_ids

    def _box_iou(self, box1, box2):
        x1, y1, x2, y2 = box1
        x3, y3, x4, y4 = box2
        xi1, yi1 = max(x1, x3), max(y1, y3)
        xi2, yi2 = min(x2, x4), min(y2, y4)
        inter_area = max(0, xi2 - xi1) * max(0, yi2 - yi1)
        box1_area = (x2 - x1) * (y2 - y1)
        box2_area = (x4 - x3) * (y4 - y3)
        union_area = box1_area + box2_area - inter_area
        return inter_area / union_area if union_area > 0 else 0

tracker = SimpleTracker(IOU_THRESHOLD)

# Generate unique filename
timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
video_path = os.path.join(VIDEO_DIR, f"recording_{timestamp}.mp4")

# Initialize camera
picam2 = Picamera2()
# Configure for preview (used for overlay) + video
preview_config = picam2.create_preview_configuration(main={"size": (1280, 720)})
picam2.configure(preview_config)
picam2.start()

# Encoder and FFmpeg output
encoder = H264Encoder(bitrate=10000000)
output = FfmpegOutput(video_path, audio=False)

# Start recording (separate from preview loop)
picam2.start_recording(encoder, output)

print(f"Recording with person detection to {video_path}")
print("Press Ctrl+C to stop...")

try:
    while True:
        # Capture array for processing (not for recording)
        frame = picam2.capture_array()

        # Preprocess for YOLO
        orig_h, orig_w = frame.shape[:2]
        input_img = cv2.resize(frame, INPUT_SIZE)
        input_img = input_img.transpose(2, 0, 1)  # HWC to CHW
        input_img = np.expand_dims(input_img, axis=0).astype(np.float32)
        input_img /= 255.0  # Normalize to [0,1]

        # Inference
        outputs = ort_session.run(None, {'images': input_img})
        detections = outputs[0][0]  # Shape: [num_boxes, 6] or [num_boxes, 5 + num_classes]

        person_boxes = []
        person_confs = []

        for det in detections:
            if len(det) == 6:
                # Format: [x_center, y_center, w, h, conf, cls]
                x_center, y_center, w, h, conf, cls_id = det
                cls_id = int(cls_id)
            else:
                # Multi-class format: [x_center, y_center, w, h, conf, ...class_probs]
                x_center, y_center, w, h, conf = det[:5]
                cls_probs = det[5:]
                cls_id = int(np.argmax(cls_probs))
                if len(cls_probs) > 1:
                    conf = conf * cls_probs[cls_id]  # Adjust confidence

            if cls_id == PERSON_CLASS_ID and conf >= CONF_THRESHOLD:
                # Denormalize coordinates
                x1 = int((x_center - w / 2) * orig_w)
                y1 = int((y_center - h / 2) * orig_h)
                x2 = int((x_center + w / 2) * orig_w)
                y2 = int((y_center + h / 2) * orig_h)
                person_boxes.append([x1, y1, x2, y2])
                person_confs.append(conf)

        # Track IDs
        ids = tracker.update(person_boxes)

        # Draw on frame (this affects preview but NOT recorded video!)
        # To draw on recorded video, we'd need to use a custom encoder or record processed frames — which is heavy.
        # Instead, we record raw video and optionally save an annotated copy separately.
        # For now: just show on preview.
        annotated = frame.copy()
        for (x1, y1, x2, y2), obj_id in zip(person_boxes, ids):
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(annotated, f"ID: {obj_id}", (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # Show preview (optional, remove if headless)
        cv2.imshow("Person Detection (Preview)", annotated)
        if cv2.waitKey(1) == ord('q'):
            break

except KeyboardInterrupt:
    print("\nStopping recording...")

finally:
    picam2.stop_recording()
    picam2.stop()
    cv2.destroyAllWindows()
    print(f"Video saved: {video_path}")