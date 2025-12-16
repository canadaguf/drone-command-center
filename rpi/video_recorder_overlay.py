"""
Standalone video recorder with overlays:
- YOLO person detection boxes with IDs
- ToF sensor distance readings
- Throttle/Yaw/Pitch/Roll telemetry

Records video with normal color and original speed.
"""

import cv2
import numpy as np
import time
import errno
import logging
import argparse
import subprocess
from datetime import datetime
from pathlib import Path
from typing import Optional, Dict, Any, List, Tuple
from picamera2 import Picamera2
import board
import busio
import adafruit_vl53l1x
from smbus2 import SMBus
from pymavlink import mavutil

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# ToF sensor configuration
MULTIPLEXER_ADDRESS = 0x70
VL53L1X_ADDRESS = 0x29
CHANNEL_FORWARD = 0
CHANNEL_DOWN = 1
MAX_I2C_RETRIES = 5
BASE_RETRY_DELAY = 0.02


class ToFSensorReader:
    """Simple ToF sensor reader using multiplexer."""
    
    def __init__(self):
        self.smbus = None
        self.i2c = None
        self.sensor_forward = None
        self.sensor_down = None
        self.initialized = False
        
    def initialize(self) -> bool:
        """Initialize ToF sensors."""
        try:
            logger.info("Initializing ToF sensors...")
            self.smbus = SMBus(1)
            self.i2c = busio.I2C(board.SCL, board.SDA, frequency=100000)
            
            # Initialize forward sensor
            try:
                self._select_channel(CHANNEL_FORWARD)
                time.sleep(0.1)
                self.sensor_forward = adafruit_vl53l1x.VL53L1X(self.i2c, address=VL53L1X_ADDRESS)
                self.sensor_forward.start_ranging()
                logger.info("✓ Forward ToF sensor initialized")
            except Exception as e:
                logger.warning(f"Failed to initialize forward sensor: {e}")
                self.sensor_forward = None
            
            # Initialize down sensor
            try:
                self._select_channel(CHANNEL_DOWN)
                time.sleep(0.1)
                self.sensor_down = adafruit_vl53l1x.VL53L1X(self.i2c, address=VL53L1X_ADDRESS)
                self.sensor_down.start_ranging()
                logger.info("✓ Down ToF sensor initialized")
            except Exception as e:
                logger.warning(f"Failed to initialize down sensor: {e}")
                self.sensor_down = None
            
            if self.sensor_forward is None and self.sensor_down is None:
                logger.error("No ToF sensors initialized")
                return False
            
            self.initialized = True
            return True
        except Exception as e:
            logger.error(f"Failed to initialize ToF sensors: {e}")
            return False
    
    def _select_channel(self, channel: int, retry_count: int = 0) -> None:
        """Select multiplexer channel with retry logic."""
        try:
            channel_mask = 1 << channel
            self.smbus.write_byte(MULTIPLEXER_ADDRESS, channel_mask)
            time.sleep(0.01)
        except OSError as e:
            if (e.errno == errno.EAGAIN or e.errno == 11) and retry_count < MAX_I2C_RETRIES:
                delay = BASE_RETRY_DELAY * (2 ** retry_count)
                time.sleep(delay)
                return self._select_channel(channel, retry_count + 1)
            else:
                raise
    
    def read_forward(self) -> Optional[float]:
        """Read forward sensor distance in meters."""
        if not self.initialized or self.sensor_forward is None:
            return None
        try:
            self._select_channel(CHANNEL_FORWARD)
            time.sleep(0.02)
            
            # Check data ready
            for attempt in range(3):
                try:
                    if self.sensor_forward.data_ready:
                        distance_cm = self.sensor_forward.distance
                        self.sensor_forward.clear_interrupt()
                        return distance_cm / 100.0  # Convert to meters
                    break
                except OSError as e:
                    if e.errno == errno.EAGAIN or e.errno == 11:
                        time.sleep(0.01 * (attempt + 1))
                        continue
                    else:
                        raise
        except Exception as e:
            logger.debug(f"Error reading forward sensor: {e}")
        return None
    
    def read_down(self) -> Optional[float]:
        """Read down sensor distance in meters."""
        if not self.initialized or self.sensor_down is None:
            return None
        try:
            self._select_channel(CHANNEL_DOWN)
            time.sleep(0.02)
            
            # Check data ready
            for attempt in range(3):
                try:
                    if self.sensor_down.data_ready:
                        distance_cm = self.sensor_down.distance
                        self.sensor_down.clear_interrupt()
                        return distance_cm / 100.0  # Convert to meters
                    break
                except OSError as e:
                    if e.errno == errno.EAGAIN or e.errno == 11:
                        time.sleep(0.01 * (attempt + 1))
                        continue
                    else:
                        raise
        except Exception as e:
            logger.debug(f"Error reading down sensor: {e}")
        return None
    
    def cleanup(self):
        """Cleanup sensors."""
        if self.sensor_forward:
            try:
                self._select_channel(CHANNEL_FORWARD)
                self.sensor_forward.stop_ranging()
            except:
                pass
        if self.sensor_down:
            try:
                self._select_channel(CHANNEL_DOWN)
                self.sensor_down.stop_ranging()
            except:
                pass
        if self.smbus:
            try:
                self.smbus.close()
            except:
                pass


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


class TelemetryReader:
    """Simple telemetry reader for throttle/yaw/pitch/roll."""
    
    def __init__(self, connection: str = "/dev/ttyAMA0", baud: int = 256000):
        self.connection_string = connection
        self.baud = baud
        self.master = None
        self.connected = False
        self.telemetry = {
            'throttle': None,
            'yaw': None,
            'pitch': None,
            'roll': None
        }
    
    def connect(self) -> bool:
        """Connect to MAVLink."""
        try:
            # Build connection string - pymavlink expects device:baud for serial
            if self.connection_string.startswith('/dev/'):
                # Serial connection: format is device:baud (no "serial:" prefix)
                conn_str = f"{self.connection_string}:{self.baud}"
            else:
                # TCP/UDP connection (use as-is)
                conn_str = self.connection_string
            
            logger.info(f"Connecting to MAVLink at {conn_str}...")
            self.master = mavutil.mavlink_connection(conn_str)
            self.master.wait_heartbeat(timeout=10)
            self.connected = True
            logger.info("Connected to MAVLink")
            return True
        except Exception as e:
            logger.warning(f"Failed to connect to MAVLink: {e}")
            self.connected = False
            return False
    
    def update(self):
        """Update telemetry from MAVLink messages."""
        if not self.connected or not self.master:
            return
        
        try:
            # Process available messages
            while True:
                msg = self.master.recv_match(timeout=0.01)
                if msg is None:
                    break
                
                msg_type = msg.get_type()
                
                if msg_type == 'ATTITUDE':
                    self.telemetry['roll'] = msg.roll
                    self.telemetry['pitch'] = msg.pitch
                    self.telemetry['yaw'] = msg.yaw
                
                elif msg_type == 'VFR_HUD':
                    self.telemetry['throttle'] = msg.throttle
        except Exception as e:
            logger.debug(f"Telemetry update error: {e}")
    
    def get_telemetry(self) -> Dict[str, Optional[float]]:
        """Get current telemetry."""
        return self.telemetry.copy()
    
    def cleanup(self):
        """Cleanup connection."""
        if self.master:
            try:
                self.master.close()
            except:
                pass
        self.connected = False


def draw_overlays(frame: np.ndarray, detections: List[Dict[str, Any]], 
                 tof_forward: Optional[float], tof_down: Optional[float],
                 telemetry: Dict[str, Optional[float]]) -> np.ndarray:
    """Draw all overlays on frame."""
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
    
    # Draw ToF sensor data (top right)
    tof_text = []
    if tof_forward is not None:
        tof_text.append(f"Forward ToF: {tof_forward:.2f}m")
    else:
        tof_text.append("Forward ToF: N/A")
    
    if tof_down is not None:
        tof_text.append(f"Down ToF: {tof_down:.2f}m")
    else:
        tof_text.append("Down ToF: N/A")
    
    y_offset = 30
    for text in tof_text:
        cv2.putText(overlay, text, (frame.shape[1] - 250, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        y_offset += 25
    
    # Draw telemetry data (bottom right)
    telemetry_text = []
    if telemetry.get('throttle') is not None:
        telemetry_text.append(f"Throttle: {telemetry['throttle']:.1f}%")
    else:
        telemetry_text.append("Throttle: N/A")
    
    if telemetry.get('yaw') is not None:
        yaw_deg = np.degrees(telemetry['yaw'])
        telemetry_text.append(f"Yaw: {yaw_deg:.1f}°")
    else:
        telemetry_text.append("Yaw: N/A")
    
    if telemetry.get('pitch') is not None:
        pitch_deg = np.degrees(telemetry['pitch'])
        telemetry_text.append(f"Pitch: {pitch_deg:.1f}°")
    else:
        telemetry_text.append("Pitch: N/A")
    
    if telemetry.get('roll') is not None:
        roll_deg = np.degrees(telemetry['roll'])
        telemetry_text.append(f"Roll: {roll_deg:.1f}°")
    else:
        telemetry_text.append("Roll: N/A")
    
    y_offset = frame.shape[0] - 100
    for text in telemetry_text:
        cv2.putText(overlay, text, (frame.shape[1] - 250, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        y_offset += 25
    
    return overlay


def main():
    parser = argparse.ArgumentParser(description="Record video with overlays")
    parser.add_argument("--model-path", default="/home/ilya/models/yolo11n.onnx",
                       help="Path to YOLO ONNX model")
    parser.add_argument("--output", default=None,
                       help="Output video path (default: timestamped)")
    parser.add_argument("--width", type=int, default=1980, help="Video width")
    parser.add_argument("--height", type=int, default=1080, help="Video height")
    parser.add_argument("--fps", type=int, default=30, help="Video FPS")
    parser.add_argument("--mavlink", default="/dev/ttyAMA0", help="MAVLink connection")
    parser.add_argument("--mavlink-baud", type=int, default=256000, help="MAVLink baud rate")
    parser.add_argument("--confidence", type=float, default=0.5, help="YOLO confidence threshold")
    
    args = parser.parse_args()
    
    # Generate output filename if not provided
    if args.output is None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_path = f"/home/ilya/videos/recording_{timestamp}.mp4"
    else:
        output_path = args.output
    
    # Create output directory if needed
    Path(output_path).parent.mkdir(parents=True, exist_ok=True)
    
    logger.info(f"Starting video recording to: {output_path}")
    logger.info(f"Resolution: {args.width}x{args.height} @ {args.fps}fps")
    
    # Initialize components
    camera = None
    yolo_detector = None
    tracker = None
    tof_reader = None
    telemetry_reader = None
    video_writer = None
    
    try:
        # Initialize camera
        logger.info("Initializing camera...")
        camera = Picamera2()
        
        # Create camera configuration with proper color settings
        # Use RGB888 format (standard format for natural colors)
        config = camera.create_preview_configuration(
            main={"size": (args.width, args.height), "format": "RGB888"},
            controls={
                "FrameRate": args.fps,
                # Color controls for natural colors (per Picamera2 documentation)
                "AwbEnable": True,  # Enable auto white balance
                "AwbMode": 0,  # Auto white balance mode (0=Auto)
                "Saturation": 1.0,  # Normal saturation (1.0 = default, range 0.0-32.0)
                "Contrast": 1.0,  # Normal contrast (1.0 = default)
                "Brightness": 0.0,  # Default brightness
                "Sharpness": 1.0  # Normal sharpness (1.0 = default, range 0.0-16.0)
            }
        )
        camera.configure(config)
        camera.start()
        time.sleep(2)  # Wait for camera to stabilize
        logger.info("Camera initialized")
        
        # Initialize YOLO detector
        logger.info("Initializing YOLO detector...")
        yolo_detector = YOLODetector(args.model_path, confidence=args.confidence)
        if yolo_detector.net is None:
            logger.error("Failed to load YOLO model - detection disabled")
            yolo_detector = None
        
        # Initialize tracker
        tracker = SimpleTracker()
        
        # Initialize ToF sensors
        logger.info("Initializing ToF sensors...")
        tof_reader = ToFSensorReader()
        tof_reader.initialize()
        
        # Initialize telemetry reader
        logger.info("Initializing telemetry reader...")
        telemetry_reader = TelemetryReader(args.mavlink, args.mavlink_baud)
        telemetry_reader.connect()
        
        # Check if FFmpeg is available, otherwise fall back to OpenCV VideoWriter
        ffmpeg_available = False
        try:
            result = subprocess.run(['which', 'ffmpeg'], capture_output=True, text=True)
            ffmpeg_available = result.returncode == 0
        except:
            pass
        
        if ffmpeg_available:
            # Initialize FFmpeg process for proper MP4 recording with timestamps
            # FFmpeg will create a proper MP4 container with correct frame rate metadata
            ffmpeg_cmd = [
                'ffmpeg',
                '-y',  # Overwrite output file
                '-f', 'rawvideo',  # Input format: raw video
                '-vcodec', 'rawvideo',
                '-s', f'{args.width}x{args.height}',  # Frame size
                '-pix_fmt', 'bgr24',  # Pixel format (BGR for OpenCV)
                '-r', str(args.fps),  # Input frame rate
                '-i', '-',  # Read from stdin
                '-an',  # No audio
                '-vcodec', 'libx264',  # H264 codec
                '-pix_fmt', 'yuv420p',  # Output pixel format (compatible with most players)
                '-preset', 'ultrafast',  # Encoding preset (ultrafast for real-time)
                '-crf', '23',  # Quality (18-28, lower = better quality)
                '-r', str(args.fps),  # Output frame rate (must match input)
                output_path
            ]
            
            logger.info(f"Using FFmpeg for recording: {' '.join(ffmpeg_cmd)}")
            ffmpeg_process = subprocess.Popen(
                ffmpeg_cmd,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            
            if ffmpeg_process.stdin is None:
                raise Exception("Failed to start FFmpeg process")
            
            video_writer = None
        else:
            # Fallback to OpenCV VideoWriter (may have speed issues, but will work)
            logger.warning("FFmpeg not found. Using OpenCV VideoWriter (video may play fast).")
            logger.warning("Install FFmpeg for proper MP4 timestamps: sudo apt-get install ffmpeg")
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            video_writer = cv2.VideoWriter(output_path, fourcc, float(args.fps), (args.width, args.height))
            
            if not video_writer.isOpened():
                raise Exception("Failed to open video writer")
            
            ffmpeg_process = None
        
        logger.info("Starting recording... Press Ctrl+C to stop")
        
        frame_count = 0
        start_time = time.time()
        frame_interval = 1.0 / args.fps  # Time between frames in seconds
        next_frame_time = start_time
        
        while True:
            # Control frame rate - wait until it's time for next frame
            current_time = time.time()
            sleep_time = next_frame_time - current_time
            if sleep_time > 0:
                time.sleep(sleep_time)
            
            # Capture frame (RGB888 format)
            frame = camera.capture_array()
            
            # Convert RGB to BGR for OpenCV (RGB888 is already RGB, no alpha channel)
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            
            # Update next frame time (use start time + frame_count * interval to prevent drift)
            next_frame_time = start_time + (frame_count + 1) * frame_interval
            
            # Run YOLO detection
            detections = []
            if yolo_detector:
                detections = yolo_detector.detect(frame_bgr)
                detections = tracker.update(detections)
            
            # Read ToF sensors
            tof_forward = tof_reader.read_forward()
            tof_down = tof_reader.read_down()
            
            # Update telemetry
            telemetry_reader.update()
            telemetry = telemetry_reader.get_telemetry()
            
            # Draw overlays
            frame_with_overlays = draw_overlays(
                frame_bgr, detections, tof_forward, tof_down, telemetry
            )
            
            # Write frame
            if ffmpeg_process and ffmpeg_process.stdin:
                # Write frame to FFmpeg stdin (proper MP4 with timestamps)
                try:
                    ffmpeg_process.stdin.write(frame_with_overlays.tobytes())
                except BrokenPipeError:
                    logger.error("FFmpeg process ended unexpectedly")
                    break
            elif video_writer:
                # Write frame using OpenCV VideoWriter (fallback)
                video_writer.write(frame_with_overlays)
            
            frame_count += 1
            if frame_count % 30 == 0:
                elapsed = time.time() - start_time
                fps_actual = frame_count / elapsed
                logger.info(f"Recorded {frame_count} frames ({fps_actual:.1f} fps, target: {args.fps} fps)")
    
    except KeyboardInterrupt:
        logger.info("Stopping recording...")
    except Exception as e:
        logger.error(f"Error during recording: {e}", exc_info=True)
    finally:
        # Cleanup
        logger.info("Cleaning up...")
        
        # Close FFmpeg stdin and wait for encoding to finish
        if 'ffmpeg_process' in locals() and ffmpeg_process and ffmpeg_process.stdin:
            logger.info("Finalizing video encoding...")
            try:
                ffmpeg_process.stdin.close()
                ffmpeg_process.wait(timeout=30)  # Wait up to 30 seconds for encoding
                
                # Check for errors
                if ffmpeg_process.returncode != 0:
                    stderr_output = ffmpeg_process.stderr.read().decode('utf-8', errors='ignore')
                    logger.warning(f"FFmpeg exited with code {ffmpeg_process.returncode}")
                    logger.debug(f"FFmpeg stderr: {stderr_output}")
                else:
                    logger.info("Video encoding completed successfully")
            except subprocess.TimeoutExpired:
                logger.warning("FFmpeg encoding timed out, killing process")
                ffmpeg_process.kill()
            except Exception as e:
                logger.error(f"Error closing FFmpeg: {e}")
        
        # Close OpenCV VideoWriter if used
        if 'video_writer' in locals() and video_writer:
            video_writer.release()
        
        if camera:
            camera.stop()
            camera.close()
        if tof_reader:
            tof_reader.cleanup()
        if telemetry_reader:
            telemetry_reader.cleanup()
        
        logger.info(f"Recording saved to: {output_path}")


if __name__ == "__main__":
    main()

