#!/usr/bin/env python3
"""
Video recorder with YOLO person detection overlays.
Based on video_tst.py - uses same recording logic for proper timestamps and colors.
"""

from picamera2 import Picamera2
from picamera2.encoders import H264Encoder
from picamera2.outputs import FfmpegOutput
import cv2
import numpy as np
import time
import logging
import argparse
from datetime import datetime
from pathlib import Path
from typing import List, Dict, Any, Tuple

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


class YOLODetector:
    """YOLO detector for person detection only."""
    
    def __init__(self, model_path: str, input_size: int = 320, confidence: float = 0.5):
        self.model_path = model_path
        self.input_size = input_size
        self.confidence_threshold = confidence
        self.target_class = 0  # Person class in COCO
        self.net = None
        self._load_model()
    
    def _load_model(self) -> bool:
        """Load YOLO ONNX model."""
        try:
            logger.info(f"Loading YOLO model from {self.model_path}")
            self.net = cv2.dnn.readNetFromONNX(self.model_path)
            self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
            self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
            logger.info("YOLO model loaded successfully")
            return True
        except Exception as e:
            logger.error(f"Failed to load YOLO model: {e}")
            return False
    
    def detect(self, frame: np.ndarray) -> List[Dict[str, Any]]:
        """Detect persons in frame."""
        if self.net is None:
            return []
        
        try:
            # Preprocess
            blob = cv2.dnn.blobFromImage(
                frame, 1/255.0, (self.input_size, self.input_size),
                mean=(0, 0, 0), swapRB=True, crop=False
            )
            
            # Inference
            self.net.setInput(blob)
            outputs = self.net.forward()
            
            if outputs is None:
                return []
            
            # Post-process
            return self._post_process(outputs, frame.shape[:2])
        except Exception as e:
            logger.debug(f"Detection error: {e}")
            return []
    
    def _post_process(self, outputs: np.ndarray, img_shape: Tuple[int, int]) -> List[Dict[str, Any]]:
        """Post-process YOLO outputs."""
        outputs = outputs[0]  # Remove batch dimension
        
        boxes = []
        scores = []
        
        x_factor = img_shape[1] / self.input_size
        y_factor = img_shape[0] / self.input_size
        
        for i in range(outputs.shape[1]):
            try:
                class_scores = outputs[4:, i]
                max_score = np.max(class_scores)
                class_id = np.argmax(class_scores)
                
                if max_score >= self.confidence_threshold and class_id == self.target_class:
                    cx, cy, w, h = outputs[0:4, i]
                    
                    if not all(np.isfinite([cx, cy, w, h])):
                        continue
                    
                    left = int((cx - w / 2) * x_factor)
                    top = int((cy - h / 2) * y_factor)
                    width = int(w * x_factor)
                    height = int(h * y_factor)
                    
                    left = max(0, min(left, img_shape[1] - 1))
                    top = max(0, min(top, img_shape[0] - 1))
                    width = max(1, min(width, img_shape[1] - left))
                    height = max(1, min(height, img_shape[0] - top))
                    
                    boxes.append([left, top, width, height])
                    scores.append(float(max_score))
            except:
                continue
        
        # Apply NMS
        if len(boxes) > 0:
            indices = cv2.dnn.NMSBoxes(boxes, scores, self.confidence_threshold, 0.4)
            
            if len(indices) > 0:
                if isinstance(indices, list):
                    indices = np.array(indices)
                
                detections = []
                for i in indices.flatten():
                    x, y, w, h = boxes[i]
                    detections.append({
                        'bbox': (x, y, x + w, y + h),
                        'confidence': scores[i],
                        'class_id': self.target_class,
                        'class_name': 'person'
                    })
                return detections
        
        return []


class SimpleTracker:
    """Simple IoU-based tracker for assigning IDs."""
    
    def __init__(self, max_age: int = 30, iou_threshold: float = 0.3):
        self.max_age = max_age
        self.iou_threshold = iou_threshold
        self.tracks = {}
        self.next_id = 1
    
    def update(self, detections: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """Update tracker and assign IDs."""
        if len(detections) == 0:
            # Age all tracks
            for track_id in list(self.tracks.keys()):
                self.tracks[track_id]['age'] += 1
                if self.tracks[track_id]['age'] > self.max_age:
                    del self.tracks[track_id]
            return []
        
        # Match detections to tracks
        track_ids = list(self.tracks.keys())
        matched_tracks = set()
        matched_detections = set()
        tracked_detections = []
        
        # Calculate IoU matrix
        for i, track_id in enumerate(track_ids):
            track_bbox = self.tracks[track_id]['bbox']
            for j, det in enumerate(detections):
                iou = self._calculate_iou(track_bbox, det['bbox'])
                if iou > self.iou_threshold and i not in matched_tracks and j not in matched_detections:
                    # Match found
                    self.tracks[track_id]['bbox'] = det['bbox']
                    self.tracks[track_id]['age'] = 0
                    det['id'] = track_id
                    tracked_detections.append(det)
                    matched_tracks.add(i)
                    matched_detections.add(j)
        
        # Create new tracks for unmatched detections
        for j, det in enumerate(detections):
            if j not in matched_detections:
                track_id = self.next_id
                self.next_id += 1
                self.tracks[track_id] = {'bbox': det['bbox'], 'age': 0}
                det['id'] = track_id
                tracked_detections.append(det)
        
        # Age unmatched tracks
        for track_id in track_ids:
            if track_id not in matched_tracks:
                self.tracks[track_id]['age'] += 1
                if self.tracks[track_id]['age'] > self.max_age:
                    del self.tracks[track_id]
        
        return tracked_detections
    
    def _calculate_iou(self, bbox1: Tuple[int, int, int, int], bbox2: Tuple[int, int, int, int]) -> float:
        """Calculate IoU between two bounding boxes."""
        x1_1, y1_1, x2_1, y2_1 = bbox1
        x1_2, y1_2, x2_2, y2_2 = bbox2
        
        x1_i = max(x1_1, x1_2)
        y1_i = max(y1_1, y1_2)
        x2_i = min(x2_1, x2_2)
        y2_i = min(y2_1, y2_2)
        
        if x2_i <= x1_i or y2_i <= y1_i:
            return 0.0
        
        intersection = (x2_i - x1_i) * (y2_i - y1_i)
        area1 = (x2_1 - x1_1) * (y2_1 - y1_1)
        area2 = (x2_2 - x1_2) * (y2_2 - y1_2)
        union = area1 + area2 - intersection
        
        return intersection / union if union > 0 else 0.0


def draw_yolo_overlays(frame: np.ndarray, detections: List[Dict[str, Any]]) -> np.ndarray:
    """Draw YOLO detection boxes with IDs on frame."""
    overlay = frame.copy()
    
    # Draw YOLO detections
    for det in detections:
        x1, y1, x2, y2 = det['bbox']
        track_id = det.get('id', 0)
        confidence = det.get('confidence', 0.0)
        
        # Draw bounding box
        cv2.rectangle(overlay, (x1, y1), (x2, y2), (0, 255, 0), 2)
        
        # Draw label with ID
        label = f"Person ID:{track_id} {confidence:.2f}"
        label_size, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
        cv2.rectangle(overlay, (x1, y1 - label_size[1] - 10),
                     (x1 + label_size[0], y1), (0, 255, 0), -1)
        cv2.putText(overlay, label, (x1, y1 - 5),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
    
    return overlay




def main():
    parser = argparse.ArgumentParser(description="Record video with YOLO person detection")
    parser.add_argument("--model-path", default="/home/ilya/models/yolo11n.onnx",
                       help="Path to YOLO ONNX model")
    parser.add_argument("--output", default=None,
                       help="Output video path (default: timestamped)")
    parser.add_argument("--confidence", type=float, default=0.5, help="YOLO confidence threshold")
    
    args = parser.parse_args()
    
    # Generate output filename if not provided
    if args.output is None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_path = f"/home/ilya/videos/yolo_recording_{timestamp}.mp4"
    else:
        output_path = args.output
    
    # Create output directory if needed
    Path(output_path).parent.mkdir(parents=True, exist_ok=True)
    
    logger.info(f"Starting video recording with YOLO detection")
    logger.info(f"Output: {output_path}")
    logger.info("Press Ctrl+C to stop")
    
    # Initialize camera (same as video_tst.py)
    picam2 = Picamera2()
    video_config = picam2.create_video_configuration()
    picam2.configure(video_config)
    picam2.start()
    time.sleep(2)  # Wait for camera to stabilize
    
    # Get camera info
    camera_config = picam2.camera_config
    width = camera_config['main']['size'][0]
    height = camera_config['main']['size'][1]
    controls = camera_config.get('controls', {})
    fps = controls.get('FrameRate', 30)
    
    logger.info(f"Video resolution: {width}x{height} @ {fps}fps")
    
    # Initialize YOLO detector
    yolo_detector = YOLODetector(args.model_path, confidence=args.confidence)
    if yolo_detector.net is None:
        logger.error("Failed to load YOLO model - continuing without detection")
        yolo_detector = None
    
    # Initialize tracker
    tracker = SimpleTracker()
    
    # Use FFmpeg with same settings as video_tst.py's H264Encoder/FfmpegOutput
    # This ensures proper timestamps - key is maintaining exact frame timing
    import subprocess
    ffmpeg_cmd = [
        'ffmpeg',
        '-y',
        '-f', 'rawvideo',
        '-vcodec', 'rawvideo',
        '-s', f'{width}x{height}',
        '-pix_fmt', 'rgb24',
        '-r', str(fps),  # Input frame rate - CRITICAL for timestamps
        '-i', '-',
        '-an',
        '-vcodec', 'libx264',
        '-pix_fmt', 'yuv420p',
        '-b:v', '10M',  # 10 Mbps (same as video_tst.py)
        '-r', str(fps),  # Output frame rate - must match input
        output_path
    ]
    
    logger.info("Starting FFmpeg encoder...")
    ffmpeg_process = subprocess.Popen(
        ffmpeg_cmd,
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE
    )
    
    if ffmpeg_process.stdin is None:
        raise Exception("Failed to start FFmpeg process")
    
    logger.info("Recording started... Press Ctrl+C to stop")
    
    try:
        frame_count = 0
        start_time = time.time()
        frame_interval = 1.0 / fps  # Time between frames
        
        while True:
            # CRITICAL: Maintain exact frame timing (like start_recording does)
            current_time = time.time()
            target_time = start_time + (frame_count * frame_interval)
            sleep_time = target_time - current_time
            if sleep_time > 0:
                time.sleep(sleep_time)
            
            # Capture frame
            frame_rgb = picam2.capture_array()
            
            # Convert to BGR for OpenCV processing
            frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
            
            # Run YOLO detection
            detections = []
            if yolo_detector:
                detections = yolo_detector.detect(frame_bgr)
                detections = tracker.update(detections)
            
            # Draw overlays
            frame_with_overlays = draw_yolo_overlays(frame_bgr, detections)
            
            # Convert back to RGB for FFmpeg
            frame_rgb_out = cv2.cvtColor(frame_with_overlays, cv2.COLOR_BGR2RGB)
            
            # Write frame to FFmpeg at exact timing
            try:
                ffmpeg_process.stdin.write(frame_rgb_out.tobytes())
            except BrokenPipeError:
                logger.error("FFmpeg process ended unexpectedly")
                break
            
            frame_count += 1
            if frame_count % 30 == 0:
                elapsed = time.time() - start_time
                actual_fps = frame_count / elapsed
                logger.info(f"Recorded {frame_count} frames ({actual_fps:.1f} fps, target: {fps} fps)")
    
    except KeyboardInterrupt:
        logger.info("Stopping recording...")
    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
    finally:
        # Close FFmpeg properly
        if ffmpeg_process and ffmpeg_process.stdin:
            logger.info("Finalizing video encoding...")
            try:
                ffmpeg_process.stdin.close()
                ffmpeg_process.wait(timeout=30)
                if ffmpeg_process.returncode == 0:
                    logger.info("Video encoding completed successfully")
                else:
                    stderr = ffmpeg_process.stderr.read().decode('utf-8', errors='ignore')
                    logger.warning(f"FFmpeg exit code: {ffmpeg_process.returncode}")
                    logger.debug(f"FFmpeg output: {stderr}")
            except subprocess.TimeoutExpired:
                logger.warning("FFmpeg encoding timed out")
                ffmpeg_process.kill()
            except Exception as e:
                logger.error(f"Error closing FFmpeg: {e}")
        
        picam2.stop()
        picam2.close()
        logger.info(f"Recording saved to: {output_path}")


if __name__ == "__main__":
    main()

