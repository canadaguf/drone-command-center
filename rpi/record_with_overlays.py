"""
Standalone video recorder with YOLO person detection, ToF sensors, and RC overlays.
Records video with detection boxes, object IDs, ToF distances, and RC controls.
"""

import argparse
import errno
import signal
import sys
import threading
import time
from pathlib import Path
from typing import List, Dict, Any, Optional, Tuple

import cv2
import numpy as np
import onnxruntime as ort
from picamera2 import Picamera2
import board
import busio
import adafruit_vl53l1x
from smbus2 import SMBus
import serial

# Configuration constants
MULTIPLEXER_ADDRESS = 0x70  # TCA9548A default address
VL53L1X_ADDRESS = 0x29  # VL53L1X sensor address
CHANNEL_FORWARD = 0  # Forward sensor channel
CHANNEL_DOWN = 1  # Down sensor channel

# I2C retry configuration
MAX_I2C_RETRIES = 5
BASE_RETRY_DELAY = 0.02  # 20ms base delay

# SBUS configuration
SBUS_FRAME_LEN = 25
SBUS_HEADER = 0x0F
SBUS_MIN = 172
SBUS_MAX = 1811

# COCO class names (person is class 0)
PERSON_CLASS_ID = 0
COCO_NAMES = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck",
    "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench",
    "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra",
    "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
    "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove",
    "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
    "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
    "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
    "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse",
    "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
    "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier",
    "toothbrush",
]


class SimpleTracker:
    """Simple IoU-based object tracker for maintaining IDs across frames."""
    
    def __init__(self, max_age: int = 30, min_hits: int = 1, iou_threshold: float = 0.3):
        self.max_age = max_age
        self.min_hits = min_hits
        self.iou_threshold = iou_threshold
        self.tracks = {}  # track_id -> track_data
        self.next_id = 1
        self.frame_count = 0
    
    def update(self, detections: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """Update tracker with new detections."""
        self.frame_count += 1
        
        if len(detections) == 0:
            # Age all tracks
            for track_id in list(self.tracks.keys()):
                self.tracks[track_id]['age'] += 1
                if self.tracks[track_id]['age'] > self.max_age:
                    del self.tracks[track_id]
            return []
        
        # Calculate IoU matrix
        track_ids = list(self.tracks.keys())
        iou_matrix = np.zeros((len(track_ids), len(detections)))
        
        for i, track_id in enumerate(track_ids):
            track_bbox = self.tracks[track_id]['bbox']
            for j, det in enumerate(detections):
                iou_matrix[i, j] = self._calculate_iou(track_bbox, det['bbox'])
        
        # Match detections to tracks (greedy matching)
        matched_tracks = set()
        matched_detections = set()
        tracked_detections = []
        
        # Sort by IoU (highest first)
        matches = []
        for i, track_id in enumerate(track_ids):
            for j in range(len(detections)):
                if iou_matrix[i, j] > self.iou_threshold:
                    matches.append((iou_matrix[i, j], i, j))
        
        matches.sort(reverse=True)
        
        for iou, i, j in matches:
            if i not in matched_tracks and j not in matched_detections:
                track_id = track_ids[i]
                # Update track
                self.tracks[track_id]['bbox'] = detections[j]['bbox']
                self.tracks[track_id]['age'] = 0
                self.tracks[track_id]['hits'] += 1
                
                # Add ID to detection
                tracked_det = detections[j].copy()
                tracked_det['id'] = track_id
                tracked_detections.append(tracked_det)
                
                matched_tracks.add(i)
                matched_detections.add(j)
        
        # Create new tracks for unmatched detections
        for j, det in enumerate(detections):
            if j not in matched_detections:
                track_id = self.next_id
                self.next_id += 1
                
                self.tracks[track_id] = {
                    'bbox': det['bbox'],
                    'age': 0,
                    'hits': 1
                }
                
                tracked_det = det.copy()
                tracked_det['id'] = track_id
                tracked_detections.append(tracked_det)
        
        # Age unmatched tracks
        for track_id in track_ids:
            if track_id not in matched_tracks:
                self.tracks[track_id]['age'] += 1
                if self.tracks[track_id]['age'] > self.max_age:
                    del self.tracks[track_id]
        
        # Return all tracked detections (no min_hits requirement for display)
        return tracked_detections
    
    def _calculate_iou(self, bbox1: Tuple[int, int, int, int], 
                      bbox2: Tuple[int, int, int, int]) -> float:
        """Calculate Intersection over Union (IoU) of two bounding boxes."""
        x1_1, y1_1, x2_1, y2_1 = bbox1
        x1_2, y1_2, x2_2, y2_2 = bbox2
        
        # Calculate intersection
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


class SBUSReader:
    """SBUS reader for RC channels."""
    
    def __init__(self, port: str = "/dev/ttyAMA0", baudrate: int = 256000, inverted: bool = True, poll_hz: int = 50):
        self.port = port
        self.baudrate = baudrate
        self.inverted = inverted
        self.poll_hz = max(10, poll_hz)
        self._ser: Optional[serial.Serial] = None
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._latest: Optional[List[int]] = None
    
    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="sbus-reader", daemon=True)
        self._thread.start()
    
    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1.5)
        self._close()
    
    def latest_channels(self) -> Optional[List[int]]:
        with self._lock:
            return list(self._latest) if self._latest else None
    
    def latest_normalized(self) -> Optional[List[float]]:
        chans = self.latest_channels()
        if chans is None:
            return None
        return [max(0.0, min(1.0, (v - SBUS_MIN) / float(SBUS_MAX - SBUS_MIN))) for v in chans]
    
    def _decode_frame(self, frame: bytes) -> Optional[List[int]]:
        """Decode 16 channel values from a 25-byte SBUS frame."""
        if len(frame) != SBUS_FRAME_LEN or frame[0] != SBUS_HEADER:
            return None
        
        chans = []
        bits = 0
        bit_index = 0
        for i in range(1, 23):
            bits |= frame[i] << bit_index
            bit_index += 8
            while bit_index >= 11 and len(chans) < 16:
                chans.append(bits & 0x7FF)
                bits >>= 11
                bit_index -= 11
        
        if len(chans) < 16:
            chans.extend([SBUS_MIN] * (16 - len(chans)))
        
        return chans[:16]
    
    def _run(self) -> None:
        interval = 1.0 / self.poll_hz
        try:
            self._ser = serial.Serial(
                self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_EVEN,
                stopbits=serial.STOPBITS_TWO,
                timeout=0.02,
            )
        except Exception:
            self._close()
            return
        
        buffer = bytearray()
        while not self._stop.is_set():
            start = time.time()
            try:
                data = self._ser.read(self._ser.in_waiting or SBUS_FRAME_LEN)
                if data:
                    buffer.extend(data)
                    while len(buffer) >= SBUS_FRAME_LEN:
                        if buffer[0] != SBUS_HEADER:
                            buffer.pop(0)
                            continue
                        frame = bytes(buffer[:SBUS_FRAME_LEN])
                        buffer = buffer[SBUS_FRAME_LEN:]
                        decoded = self._decode_frame(frame)
                        if decoded:
                            with self._lock:
                                self._latest = decoded
            except Exception:
                pass
            
            elapsed = time.time() - start
            sleep_for = max(0.0, interval - elapsed)
            time.sleep(sleep_for)
        
        self._close()
    
    def _close(self) -> None:
        try:
            if self._ser and self._ser.is_open:
                self._ser.close()
        except Exception:
            pass


class ToFSensorReader:
    """ToF sensor reader using multiplexer."""
    
    def __init__(self):
        self.smbus = None
        self.i2c = None
        self.sensor_forward = None
        self.sensor_down = None
        self.forward_distance = None
        self.down_distance = None
        self.initialized = False
    
    def initialize(self) -> bool:
        """Initialize ToF sensors."""
        try:
            print("Initializing ToF sensors...")
            self.smbus = SMBus(1)
            self.i2c = busio.I2C(board.SCL, board.SDA, frequency=100000)
            
            # Initialize forward sensor
            try:
                self._select_channel(CHANNEL_FORWARD)
                time.sleep(0.1)
                self.sensor_forward = adafruit_vl53l1x.VL53L1X(self.i2c, address=VL53L1X_ADDRESS)
                self.sensor_forward.start_ranging()
                print(f"✓ Forward sensor initialized on channel {CHANNEL_FORWARD}")
            except Exception as e:
                print(f"✗ Failed to initialize forward sensor: {e}")
                self.sensor_forward = None
            
            # Initialize down sensor
            try:
                self._select_channel(CHANNEL_DOWN)
                time.sleep(0.1)
                self.sensor_down = adafruit_vl53l1x.VL53L1X(self.i2c, address=VL53L1X_ADDRESS)
                self.sensor_down.start_ranging()
                print(f"✓ Down sensor initialized on channel {CHANNEL_DOWN}")
            except Exception as e:
                print(f"✗ Failed to initialize down sensor: {e}")
                self.sensor_down = None
            
            if self.sensor_forward is None and self.sensor_down is None:
                print("Warning: No ToF sensors initialized")
                return False
            
            self.initialized = True
            return True
        except Exception as e:
            print(f"Failed to initialize ToF sensors: {e}")
            return False
    
    def _select_channel(self, channel: int, retry_count: int = 0):
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
    
    def read_distances(self):
        """Read distances from both sensors."""
        # Read forward sensor
        if self.sensor_forward:
            try:
                self._select_channel(CHANNEL_FORWARD)
                time.sleep(0.02)
                
                data_ready = False
                for attempt in range(3):
                    try:
                        data_ready = self.sensor_forward.data_ready
                        break
                    except OSError as e:
                        if e.errno == errno.EAGAIN or e.errno == 11:
                            time.sleep(0.01 * (attempt + 1))
                            continue
                        else:
                            raise
                
                if data_ready:
                    distance_cm = None
                    for attempt in range(3):
                        try:
                            distance_cm = self.sensor_forward.distance
                            break
                        except OSError as e:
                            if e.errno == errno.EAGAIN or e.errno == 11:
                                time.sleep(0.01 * (attempt + 1))
                                continue
                            else:
                                raise
                    
                    if distance_cm is not None:
                        self.forward_distance = distance_cm / 100.0  # Convert to meters
                        try:
                            self.sensor_forward.clear_interrupt()
                        except:
                            pass
            except:
                pass
        
        # Read down sensor
        if self.sensor_down:
            try:
                self._select_channel(CHANNEL_DOWN)
                time.sleep(0.02)
                
                data_ready = False
                for attempt in range(3):
                    try:
                        data_ready = self.sensor_down.data_ready
                        break
                    except OSError as e:
                        if e.errno == errno.EAGAIN or e.errno == 11:
                            time.sleep(0.01 * (attempt + 1))
                            continue
                        else:
                            raise
                
                if data_ready:
                    distance_cm = None
                    for attempt in range(3):
                        try:
                            distance_cm = self.sensor_down.distance
                            break
                        except OSError as e:
                            if e.errno == errno.EAGAIN or e.errno == 11:
                                time.sleep(0.01 * (attempt + 1))
                                continue
                            else:
                                raise
                    
                    if distance_cm is not None:
                        self.down_distance = distance_cm / 100.0  # Convert to meters
                        try:
                            self.sensor_down.clear_interrupt()
                        except:
                            pass
            except:
                pass
    
    def cleanup(self):
        """Cleanup sensors."""
        if self.sensor_forward:
            try:
                self._select_channel(CHANNEL_FORWARD)
                time.sleep(0.01)
                self.sensor_forward.stop_ranging()
            except:
                pass
        
        if self.sensor_down:
            try:
                self._select_channel(CHANNEL_DOWN)
                time.sleep(0.01)
                self.sensor_down.stop_ranging()
            except:
                pass
        
        if self.smbus:
            try:
                self.smbus.close()
            except:
                pass


def parse_onnx_detections(
    outputs: np.ndarray,
    img_shape: Tuple[int, int],
    conf_thres: float,
    iou_thres: float,
    max_det: int,
) -> List[Dict[str, Any]]:
    """Parse YOLO ONNX outputs and filter for person class only."""
    detections: List[Dict[str, Any]] = []
    if outputs is None or len(outputs) == 0:
        return detections
    
    preds = outputs[0]  # [84, N]
    num_candidates = preds.shape[1]
    x_factor = img_shape[1] / 320.0
    y_factor = img_shape[0] / 320.0
    
    boxes: List[List[int]] = []
    scores: List[float] = []
    
    for i in range(num_candidates):
        # Check person class (class 0) confidence
        person_conf = float(preds[4 + PERSON_CLASS_ID, i])
        if person_conf < conf_thres:
            continue
        
        cx, cy, w, h = preds[0:4, i]
        left = int((cx - w / 2) * x_factor)
        top = int((cy - h / 2) * y_factor)
        width = int(w * x_factor)
        height = int(h * y_factor)
        
        left = max(0, min(left, img_shape[1] - 1))
        top = max(0, min(top, img_shape[0] - 1))
        width = max(1, min(width, img_shape[1] - left))
        height = max(1, min(height, img_shape[0] - top))
        
        boxes.append([left, top, width, height])
        scores.append(person_conf)
    
    dets: List[Dict[str, Any]] = []
    if boxes:
        indices = cv2.dnn.NMSBoxes(boxes, scores, conf_thres, iou_thres)
        if len(indices) > 0:
            indices = np.array(indices).flatten().tolist()
            for idx, i in enumerate(indices[:max_det]):
                x, y, w, h = boxes[i]
                dets.append({
                    "bbox": (x, y, x + w, y + h),
                    "cls": "person",
                    "conf": scores[i],
                })
    return dets


def draw_detections(frame: np.ndarray, detections: List[Dict[str, Any]]) -> None:
    """Draw detection boxes with IDs on frame."""
    for det in detections:
        x1, y1, x2, y2 = det["bbox"]
        track_id = det.get("id", 0)
        conf = det["conf"]
        label = f"person #{track_id} {conf:.2f}"
        
        # Draw bounding box
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        
        # Draw label background
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
        cv2.rectangle(frame, (x1, y1 - th - 6), (x1 + tw, y1), (0, 255, 0), -1)
        cv2.putText(frame, label, (x1, y1 - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2, cv2.LINE_AA)


def draw_overlay_text(
    frame: np.ndarray,
    text: str,
    position: Tuple[int, int],
    font_scale: float = 0.6,
    thickness: int = 2,
    color: Tuple[int, int, int] = (0, 255, 0),
    bg_color: Tuple[int, int, int] = (0, 0, 0),
    bg_alpha: float = 0.6,
):
    """Draw text with semi-transparent background."""
    font = cv2.FONT_HERSHEY_SIMPLEX
    (tw, th), _ = cv2.getTextSize(text, font, font_scale, thickness)
    x, y = position
    
    # Draw background
    overlay = frame.copy()
    cv2.rectangle(overlay, (x, y - th - 4), (x + tw + 4, y + 4), bg_color, -1)
    cv2.addWeighted(overlay, bg_alpha, frame, 1 - bg_alpha, 0, dst=frame)
    
    # Draw text
    cv2.putText(frame, text, (x + 2, y - 2), font, font_scale, color, thickness, cv2.LINE_AA)


def main():
    parser = argparse.ArgumentParser(description="Record video with YOLO, ToF, and RC overlays")
    parser.add_argument("--model", type=str, default="/home/ilya/models/yolo11n.onnx",
                       help="Path to YOLO ONNX model")
    parser.add_argument("--output", type=str, default="recording.mp4",
                       help="Output video path")
    parser.add_argument("--width", type=int, default=1280, help="Video width")
    parser.add_argument("--height", type=int, default=720, help="Video height")
    parser.add_argument("--fps", type=int, default=30, help="Video FPS")
    parser.add_argument("--conf", type=float, default=0.4, help="YOLO confidence threshold")
    parser.add_argument("--iou", type=float, default=0.45, help="YOLO IoU threshold")
    parser.add_argument("--sbus-port", type=str, default="/dev/ttyAMA0", help="SBUS port")
    parser.add_argument("--sbus-baudrate", type=int, default=256000, help="SBUS baudrate")
    parser.add_argument("--throttle-ch", type=int, default=2, help="Throttle channel (0-indexed)")
    parser.add_argument("--yaw-ch", type=int, default=3, help="Yaw channel (0-indexed)")
    parser.add_argument("--pitch-ch", type=int, default=1, help="Pitch channel (0-indexed)")
    parser.add_argument("--roll-ch", type=int, default=0, help="Roll channel (0-indexed)")
    
    args = parser.parse_args()
    
    # Initialize components
    print("Initializing components...")
    
    # YOLO model
    print(f"Loading YOLO model from {args.model}...")
    try:
        session = ort.InferenceSession(args.model, providers=["CPUExecutionProvider"])
        input_name = session.get_inputs()[0].name
        print("✓ YOLO model loaded")
    except Exception as e:
        print(f"✗ Failed to load YOLO model: {e}")
        return
    
    # ToF sensors
    tof_reader = ToFSensorReader()
    tof_reader.initialize()
    
    # SBUS reader
    print("Initializing SBUS reader...")
    try:
        sbus_reader = SBUSReader(
            port=args.sbus_port,
            baudrate=args.sbus_baudrate,
            inverted=True,
            poll_hz=50,
        )
        sbus_reader.start()
        print("✓ SBUS reader started")
    except Exception as e:
        print(f"✗ Failed to start SBUS reader: {e}")
        sbus_reader = None
    
    # Object tracker
    tracker = SimpleTracker(max_age=30, min_hits=1, iou_threshold=0.3)
    
    # Camera
    print("Initializing camera...")
    picam2 = Picamera2()
    cfg = picam2.create_preview_configuration(
        main={"size": (args.width, args.height), "format": "RGB888"},
        controls={
            "FrameRate": args.fps,
            "AwbEnable": True,
            "AwbMode": 3,  # Daylight mode (less blue)
            "Saturation": 1.0,
        },
    )
    picam2.configure(cfg)
    picam2.start()
    time.sleep(1.5)  # Warm-up
    print("✓ Camera initialized")
    
    # Video writer
    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = cv2.VideoWriter(str(output_path), fourcc, args.fps, (args.width, args.height))
    print(f"✓ Video writer initialized: {output_path}")
    
    # Signal handler
    stop_flag = False
    
    def handle_sigint(signum, frame):
        nonlocal stop_flag
        stop_flag = True
    
    signal.signal(signal.SIGINT, handle_sigint)
    
    print("\n" + "="*60)
    print("Recording started. Press Ctrl+C to stop.")
    print("="*60 + "\n")
    
    frame_count = 0
    frame_time = 1.0 / args.fps  # Time per frame in seconds
    last_frame_time = time.time()
    
    try:
        while not stop_flag:
            # Control frame rate
            current_time = time.time()
            elapsed = current_time - last_frame_time
            if elapsed < frame_time:
                time.sleep(frame_time - elapsed)
            
            last_frame_time = time.time()
            
            # Capture frame
            frame_rgb = picam2.capture_array()
            if frame_rgb is None:
                continue
            
            frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
            
            # Run YOLO detection
            resized = cv2.resize(frame_rgb, (320, 320), interpolation=cv2.INTER_LINEAR)
            blob = resized.astype(np.float32) / 255.0
            blob = np.transpose(blob, (2, 0, 1))  # HWC -> CHW
            blob = np.expand_dims(blob, 0)  # add batch
            
            outputs = session.run(None, {input_name: blob})
            detections = parse_onnx_detections(
                outputs[0], frame_rgb.shape[:2], args.conf, args.iou, max_det=10
            )
            
            # Update tracker
            tracked_detections = tracker.update(detections)
            
            # Draw detections
            draw_detections(frame_bgr, tracked_detections)
            
            # Read ToF sensors
            tof_reader.read_distances()
            
            # Read RC channels
            rc_data = {}
            if sbus_reader:
                channels = sbus_reader.latest_normalized()
                if channels:
                    rc_data = {
                        "throttle": channels[args.throttle_ch] if args.throttle_ch < len(channels) else None,
                        "yaw": channels[args.yaw_ch] if args.yaw_ch < len(channels) else None,
                        "pitch": channels[args.pitch_ch] if args.pitch_ch < len(channels) else None,
                        "roll": channels[args.roll_ch] if args.roll_ch < len(channels) else None,
                    }
            
            # Draw overlays
            y_offset = 10
            line_height = 25
            
            # ToF overlay (top right)
            tof_lines = []
            if tof_reader.forward_distance is not None:
                tof_lines.append(f"Forward ToF: {tof_reader.forward_distance:.2f}m")
            if tof_reader.down_distance is not None:
                tof_lines.append(f"Down ToF: {tof_reader.down_distance:.2f}m")
            
            for i, line in enumerate(tof_lines):
                x = args.width - 200
                y = y_offset + i * line_height
                draw_overlay_text(frame_bgr, line, (x, y))
            
            # RC overlay (bottom right)
            rc_lines = []
            if rc_data.get("throttle") is not None:
                rc_lines.append(f"Throttle: {rc_data['throttle']:.2f}")
            if rc_data.get("yaw") is not None:
                rc_lines.append(f"Yaw: {rc_data['yaw']:.2f}")
            if rc_data.get("pitch") is not None:
                rc_lines.append(f"Pitch: {rc_data['pitch']:.2f}")
            if rc_data.get("roll") is not None:
                rc_lines.append(f"Roll: {rc_data['roll']:.2f}")
            
            for i, line in enumerate(rc_lines):
                x = args.width - 200
                y = args.height - (len(rc_lines) - i) * line_height - 10
                draw_overlay_text(frame_bgr, line, (x, y))
            
            # Write frame
            writer.write(frame_bgr)
            
            frame_count += 1
            if frame_count % 30 == 0:
                print(f"Recorded {frame_count} frames...")
    
    except KeyboardInterrupt:
        print("\nStopping recording...")
    finally:
        # Cleanup
        print("Cleaning up...")
        writer.release()
        picam2.stop()
        picam2.close()
        tof_reader.cleanup()
        if sbus_reader:
            sbus_reader.stop()
        print(f"Recording saved to: {output_path}")
        print(f"Total frames: {frame_count}")


if __name__ == "__main__":
    main()

