#!/usr/bin/env python3
"""
Autonomous STABILIZE-mode follow script.

This script mirrors the standalone liftoff logic (arming, throttle ramp, ToF-based altitude hold),
then activates a vision pipeline (Picamera2 + YOLO + simple box tracking) to follow a single object class.
While following, it keeps altitude between 0.5-1.5 m using the bottom ToF sensor, estimates forward distance
mostly from bounding boxes, and blends in the forward ToF sensor whenever the target is centered.
If the target is lost and not reacquired within the timeout, the drone descends and lands using the same
incremental throttle logic validated in the liftoff tests.
"""

import argparse
import collections
import collections.abc
import errno
import logging
import signal
import sys
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

if not hasattr(collections, "MutableMapping"):
    collections.MutableMapping = collections.abc.MutableMapping  # type: ignore[attr-defined]

try:
    from picamera2 import Picamera2  # type: ignore
except ImportError:  # pragma: no cover - hardware specific
    Picamera2 = None  # type: ignore

import cv2  # type: ignore
import numpy as np  # type: ignore
from dronekit import VehicleMode, connect  # type: ignore

import board  # type: ignore
import busio  # type: ignore
import adafruit_vl53l1x  # type: ignore
from smbus2 import SMBus  # type: ignore

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

CONNECTION_STRING = "/dev/ttyAMA0"
BAUD_RATE = 256000
TARGET_ALTITUDE = 1.0
MIN_ALTITUDE = 0.5
MAX_ALTITUDE = 1.5
GROUND_THRESHOLD = 0.08
TAKEOFF_THRESHOLD = 0.1
ALTITUDE_TOLERANCE = 0.05
THROTTLE_IDLE = 1100
THROTTLE_MAX = 1900
HOVER_THROTTLE = 1500
THROTTLE_STEP_UP = 6
THROTTLE_STEP_DOWN = 6

FRONT_SENSOR_CHANNEL = 0
BOTTOM_SENSOR_CHANNEL = 1
MULTIPLEXER_ADDRESS = 0x70
VL53L1X_ADDRESS = 0x29
MAX_I2C_RETRIES = 5
BASE_RETRY_DELAY = 0.02
HEIGHT_LOG_INTERVAL = 1.0

DEFAULT_MODEL_PATH = "/home/ilya/models/yolo11n.onnx"
DEFAULT_TARGET_CLASS = "chair"
COCO_CLASS_IDS = {"person": 0, "chair": 56, "bench": 14}
DESIRED_DISTANCE = 3.0
DISTANCE_MIN = 0.8
DISTANCE_MAX = 5.0
KP_DISTANCE = 0.35
KP_YAW = 0.45
KP_PITCH = 0.25
KP_ALTITUDE = 1.2

FOLLOW_DURATION = 60
LOSS_TIMEOUT = 5.0
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
FRAME_RATE = 20

logger = logging.getLogger("follow_stabilize")
LAST_HEIGHT_LOG = 0.0


def clamp(value: float, min_value: float, max_value: float) -> float:
    return max(min_value, min(max_value, value))


# ---------------------------------------------------------------------------
# ToF helper
# ---------------------------------------------------------------------------

class ToFSensor:
    def __init__(self, channel: int, log_height: bool = False):
        self.channel = channel
        self.log_height = log_height
        self.smbus: Optional[SMBus] = None
        self.i2c: Optional[busio.I2C] = None
        self.sensor: Optional["adafruit_vl53l1x.VL53L1X"] = None
        self.initialized = False

    def initialize(self) -> bool:
        try:
            self.smbus = SMBus(1)
            self.i2c = busio.I2C(board.SCL, board.SDA, frequency=100000)
            self._select_channel(self.channel)
            time.sleep(0.1)
            self.sensor = adafruit_vl53l1x.VL53L1X(self.i2c, address=VL53L1X_ADDRESS)  # type: ignore[arg-type]
            self.sensor.start_ranging()
            self.initialized = True
            logger.info("ToF sensor initialised on channel %s", self.channel)
            return True
        except Exception as exc:  # pragma: no cover
            logger.error("Failed to initialise ToF sensor on channel %s: %s", self.channel, exc)
            return False

    def _select_channel(self, channel: int, retry: int = 0) -> None:
        if not self.smbus:
            raise RuntimeError("SMBus not initialized")
        try:
            self.smbus.write_byte(MULTIPLEXER_ADDRESS, 1 << channel)
            time.sleep(0.01)
        except OSError as exc:
            if (exc.errno == errno.EAGAIN or exc.errno == 11) and retry < MAX_I2C_RETRIES:
                delay = BASE_RETRY_DELAY * (2 ** retry)
                time.sleep(delay)
                self._select_channel(channel, retry + 1)
            else:
                raise

    def get_distance(self) -> Optional[float]:
        if not self.initialized or not self.sensor or not self.smbus:
            return None
        try:
            self._select_channel(self.channel)
            time.sleep(0.02)
            ready = False
            for attempt in range(3):
                try:
                    ready = self.sensor.data_ready  # type: ignore[attr-defined]
                    break
                except OSError as exc:
                    if exc.errno in (errno.EAGAIN, 11):
                        time.sleep(0.01 * (attempt + 1))
                        continue
                    raise
            if not ready:
                return None
            distance_cm = None
            for attempt in range(3):
                try:
                    distance_cm = self.sensor.distance  # type: ignore[attr-defined]
                    break
                except OSError as exc:
                    if exc.errno in (errno.EAGAIN, 11):
                        time.sleep(0.01 * (attempt + 1))
                        continue
                    raise
            if distance_cm is None:
                return None
            try:
                self.sensor.clear_interrupt()  # type: ignore[attr-defined]
            except OSError:
                pass
            distance_m = distance_cm / 100.0
            if distance_m < 0.04 or distance_m > 4.0:
                return None
            if self.log_height:
                log_height(distance_m)
            return distance_m
        except Exception as exc:
            logger.debug("ToF read error on channel %s: %s", self.channel, exc)
            return None

    def cleanup(self) -> None:
        if self.sensor:
            try:
                self._select_channel(self.channel)
                time.sleep(0.01)
                self.sensor.stop_ranging()  # type: ignore[attr-defined]
            except Exception:
                pass
        if self.smbus:
            try:
                self.smbus.close()
            except Exception:
                pass
        self.initialized = False


def log_height(distance: float) -> None:
    global LAST_HEIGHT_LOG
    now = time.time()
    if now - LAST_HEIGHT_LOG >= HEIGHT_LOG_INTERVAL:
        logger.info("Current height: %.2fm", distance)
        LAST_HEIGHT_LOG = now


# ---------------------------------------------------------------------------
# Camera + detection helpers
# ---------------------------------------------------------------------------

class CameraManager:
    def __init__(self, width: int, height: int, fps: int):
        self.width = width
        self.height = height
        self.fps = fps
        self.picam2: Optional[Picamera2] = None
        self.initialized = False

    def initialize(self) -> bool:
        if Picamera2 is None:
            logger.error("Picamera2 not available")
            return False
        try:
            self.picam2 = Picamera2()
            config = self.picam2.create_preview_configuration(
                main={"size": (self.width, self.height), "format": "RGB888"},
                controls={"FrameRate": self.fps},
            )
            self.picam2.configure(config)
            self.picam2.start()
            time.sleep(2)
            self.initialized = True
            logger.info("Camera initialized (%sx%s @ %sfps)", self.width, self.height, self.fps)
            return True
        except Exception as exc:
            logger.error("Camera initialization failed: %s", exc)
            return False

    def capture_frame(self) -> Optional[np.ndarray]:
        if not self.initialized or not self.picam2:
            return None
        try:
            return self.picam2.capture_array()
        except Exception as exc:
            logger.error("Camera capture failed: %s", exc)
            return None

    def cleanup(self) -> None:
        if self.picam2:
            try:
                self.picam2.stop()
                self.picam2.close()
            except Exception:
                pass
        self.initialized = False


class YOLODetector:
    def __init__(self, model_path: str, input_size: int, confidence: float, nms: float, target_class_id: int):
        self.model_path = model_path
        self.input_size = input_size
        self.confidence = confidence
        self.nms = nms
        self.target_class_id = target_class_id
        self.net = None
        self._load_model()

    def _load_model(self) -> None:
        try:
            logger.info("Loading YOLO model from %s", self.model_path)
            self.net = cv2.dnn.readNetFromONNX(self.model_path)
            self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
            self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
        except Exception as exc:
            logger.error("Failed to load YOLO model: %s", exc)
            self.net = None

    def detect(self, frame: np.ndarray) -> List[Dict[str, any]]:
        if self.net is None:
            return []
        blob = cv2.dnn.blobFromImage(
            frame,
            scalefactor=1 / 255.0,
            size=(self.input_size, self.input_size),
            mean=(0, 0, 0),
            swapRB=True,
            crop=False,
        )
        self.net.setInput(blob)
        try:
            outputs = self.net.forward()
        except Exception as exc:
            logger.error("Inference failed: %s", exc)
            return []
        if outputs is None or len(outputs) == 0:
            return []
        outputs = outputs[0]
        boxes: List[List[int]] = []
        scores: List[float] = []
        class_ids: List[int] = []

        h, w = frame.shape[:2]
        x_factor = w / self.input_size
        y_factor = h / self.input_size

        for i in range(outputs.shape[1]):
            class_scores = outputs[4:, i]
            class_id = int(np.argmax(class_scores))
            confidence = class_scores[class_id]
            if class_id != self.target_class_id or confidence < self.confidence:
                continue
            cx, cy, bw, bh = outputs[0:4, i]
            left = int((cx - bw / 2) * x_factor)
            top = int((cy - bh / 2) * y_factor)
            width = int(bw * x_factor)
            height = int(bh * y_factor)
            left = max(0, min(left, w - 1))
            top = max(0, min(top, h - 1))
            width = max(1, min(width, w - left))
            height = max(1, min(height, h - top))
            boxes.append([left, top, width, height])
            scores.append(float(confidence))
            class_ids.append(class_id)

        detections: List[Dict[str, any]] = []
        if boxes:
            indices = cv2.dnn.NMSBoxes(boxes, scores, self.confidence, self.nms)
            for idx in indices:
                i = idx[0] if isinstance(idx, (list, tuple, np.ndarray)) else idx
                x, y, width, height = boxes[i]
                detections.append(
                    {
                        "bbox": (x, y, x + width, y + height),
                        "confidence": scores[i],
                        "class_id": class_ids[i],
                        "center": (x + width // 2, y + height // 2),
                    }
                )
        return detections


def bbox_iou(a: Tuple[int, int, int, int], b: Tuple[int, int, int, int]) -> float:
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b
    inter_x1 = max(ax1, bx1)
    inter_y1 = max(ay1, by1)
    inter_x2 = min(ax2, bx2)
    inter_y2 = min(ay2, by2)
    if inter_x2 <= inter_x1 or inter_y2 <= inter_y1:
        return 0.0
    intersection = (inter_x2 - inter_x1) * (inter_y2 - inter_y1)
    area_a = (ax2 - ax1) * (ay2 - ay1)
    area_b = (bx2 - bx1) * (by2 - by1)
    union = area_a + area_b - intersection
    return intersection / union if union > 0 else 0.0


def estimate_distance_from_bbox(bbox: Tuple[int, int, int, int], frame_shape: Tuple[int, int]) -> float:
    h, w = frame_shape
    x1, y1, x2, y2 = bbox
    height_ratio = (y2 - y1) / max(1, h)
    width_ratio = (x2 - x1) / max(1, w)
    height_distance = clamp(0.9 / max(height_ratio, 0.01), 0.5, 8.0)
    width_distance = clamp(0.6 / max(width_ratio, 0.01), 0.5, 8.0)
    return (height_distance + width_distance) / 2.0


# ---------------------------------------------------------------------------
# DroneKit helpers (subset)
# ---------------------------------------------------------------------------

def set_mode(vehicle, mode: str, timeout: float = 5.0) -> bool:
    logger.info("Setting mode to %s", mode)
    vehicle.mode = VehicleMode(mode)
    start = time.time()
    while time.time() - start < timeout:
        if vehicle.mode.name == mode:
            return True
        time.sleep(0.2)
    logger.error("Failed to set mode %s", mode)
    return False


def arm_vehicle(vehicle, timeout: float = 10.0) -> bool:
    logger.info("Arming vehicle...")
    vehicle.armed = True
    start = time.time()
    while time.time() - start < timeout:
        if vehicle.armed:
            logger.info("Vehicle armed")
            return True
        time.sleep(0.2)
    logger.error("Arming timed out")
    return False


def apply_manual_override(vehicle, roll: int, pitch: int, throttle: int, yaw: int) -> None:
    vehicle.channels.overrides = {
        "1": clamp(roll, 1100, 1900),
        "2": clamp(pitch, 1100, 1900),
        "3": clamp(throttle, 1100, 1900),
        "4": clamp(yaw, 1100, 1900),
    }


def clear_overrides(vehicle) -> None:
    vehicle.channels.overrides = {}


def ramp_until_takeoff(vehicle, bottom_sensor: ToFSensor, idle_pwm: int, max_pwm: int, threshold: float) -> Optional[int]:
    logger.info("Ramping throttle until ToF exceeds %.2fm", threshold)
    throttle = idle_pwm
    while throttle <= max_pwm:
        apply_manual_override(vehicle, 1500, 1500, throttle, 1500)
        time.sleep(0.25)
        distance = bottom_sensor.get_distance()
        if distance is not None and distance > threshold:
            logger.info("Takeoff detected at %.2fm (throttle %s)", distance, throttle)
            return throttle
        throttle += 3
    logger.error("Failed to detect liftoff")
    return None


def altitude_pid_loop(
    vehicle,
    bottom_sensor: ToFSensor,
    target_alt: float,
    tolerance: float,
    gains: Dict[str, float],
    throttle_pwm: int,
    duration: Optional[float] = None,
    timeout: float = 30.0,
) -> int:
    integral = 0.0
    last_error = 0.0
    dt = 0.2
    start = time.time()
    hold_start = None
    step_up = gains.get("step_up", THROTTLE_STEP_UP)
    step_down = gains.get("step_down", THROTTLE_STEP_DOWN)

    while time.time() - start < timeout:
        distance = bottom_sensor.get_distance()
        if distance is None:
            apply_manual_override(vehicle, 1500, 1500, throttle_pwm, 1500)
            time.sleep(dt)
            continue
        error = target_alt - distance
        integral += error * dt
        integral = clamp(integral, -0.5, 0.5)
        derivative = (error - last_error) / dt
        last_error = error

        adjustment = (gains["kp"] * error + gains["ki"] * integral + gains["kd"] * derivative) * 0.1
        if distance < target_alt - tolerance:
            throttle_pwm = min(gains["max_pwm"], throttle_pwm + step_up)
        elif distance > target_alt + tolerance:
            throttle_pwm = max(gains["min_pwm"], throttle_pwm - step_down)
        else:
            throttle_pwm += int(adjustment)
        throttle_pwm = clamp(throttle_pwm, gains["min_pwm"], gains["max_pwm"])
        apply_manual_override(vehicle, 1500, 1500, throttle_pwm, 1500)

        if duration is None:
            if abs(error) <= tolerance:
                if hold_start is None:
                    hold_start = time.time()
                elif time.time() - hold_start > 0.8:
                    logger.info("Target altitude reached (%.2fm)", distance)
                    return throttle_pwm
            else:
                hold_start = None
        else:
            if hold_start is None:
                hold_start = time.time()
            if time.time() - hold_start >= duration:
                logger.info("Hold complete (%.1fs)", duration)
                return throttle_pwm

        time.sleep(dt)
    logger.warning("Altitude loop timeout")
    return throttle_pwm


def descend_to_ground(vehicle, bottom_sensor: ToFSensor, throttle_pwm: int) -> None:
    logger.info("Starting descent")
    current_throttle = throttle_pwm
    start = time.time()
    while time.time() - start < 60:
        distance = bottom_sensor.get_distance()
        if distance is not None and distance <= GROUND_THRESHOLD * 2:
            logger.info("Within %.2fm of ground", distance)
            break
        reduction = 4 if (distance is None or distance > 0.5) else 2
        current_throttle = max(THROTTLE_IDLE, current_throttle - reduction)
        apply_manual_override(vehicle, 1500, 1500, current_throttle, 1500)
        time.sleep(0.2)


def final_touchdown(vehicle, bottom_sensor: ToFSensor) -> None:
    logger.info("Final landing")
    throttle = THROTTLE_IDLE + 50
    while True:
        distance = bottom_sensor.get_distance()
        if distance is not None and distance <= GROUND_THRESHOLD:
            logger.info("Touchdown detected at %.2fm", distance)
            break
        throttle = max(THROTTLE_IDLE, throttle - 2)
        apply_manual_override(vehicle, 1500, 1500, throttle, 1500)
        time.sleep(0.2)
    apply_manual_override(vehicle, 1500, 1500, THROTTLE_IDLE, 1500)
    time.sleep(0.5)
    clear_overrides(vehicle)


def disarm_vehicle(vehicle) -> None:
    logger.info("Disarming vehicle")
    vehicle.armed = False
    start = time.time()
    while vehicle.armed and time.time() - start < 5.0:
        time.sleep(0.2)


# ---------------------------------------------------------------------------
# Follow controller
# ---------------------------------------------------------------------------

@dataclass
class ControlCommand:
    vx: float
    vz: float
    yaw_rate: float


def compute_follow_command(
    bbox: Tuple[int, int, int, int],
    frame_shape: Tuple[int, int],
    target_distance: float,
    current_distance: float,
    altitude: float,
    target_altitude: float,
) -> ControlCommand:
    h, w = frame_shape
    x1, y1, x2, y2 = bbox
    center_x = (x1 + x2) / 2.0
    center_y = (y1 + y2) / 2.0

    error_x_norm = (center_x - w / 2) / w
    error_y_norm = (center_y - h / 2) / h
    yaw_rate = -KP_YAW * error_x_norm

    distance_error = current_distance - target_distance
    vx = clamp(-KP_DISTANCE * distance_error, -0.8, 0.8)

    altitude_error = clamp(target_altitude - altitude, -0.6, 0.6)
    vertical_bias = -KP_PITCH * error_y_norm
    vz = clamp(KP_ALTITUDE * altitude_error + vertical_bias * 0.3, -0.5, 0.5)

    return ControlCommand(vx=vx, vz=vz, yaw_rate=yaw_rate)


def blend_distance(bbox_distance: float, tof_distance: Optional[float], error_x: float, error_y: float) -> float:
    if tof_distance is None:
        return bbox_distance
    if abs(error_x) < 0.1 and abs(error_y) < 0.1:
        return (bbox_distance * 0.4) + (tof_distance * 0.6)
    return bbox_distance


def send_stabilize_command(vehicle, command: ControlCommand, throttle_pwm: int) -> None:
    pitch_scale = 250
    yaw_scale = 250
    pitch_pwm = int(1500 - command.vx * pitch_scale)
    yaw_pwm = int(1500 + command.yaw_rate * yaw_scale)
    throttle = clamp(throttle_pwm + int(command.vz * 50), THROTTLE_IDLE, THROTTLE_MAX)
    apply_manual_override(vehicle, roll=1500, pitch=pitch_pwm, throttle=throttle, yaw=yaw_pwm)


def follow_target_loop(
    vehicle,
    camera: CameraManager,
    detector: YOLODetector,
    front_sensor: ToFSensor,
    bottom_sensor: ToFSensor,
    hover_throttle: int,
    follow_seconds: int,
    loss_timeout: float,
) -> bool:
    start_time = time.time()
    target_bbox: Optional[Tuple[int, int, int, int]] = None
    last_seen = 0.0
    throttle_pwm = hover_throttle

    while time.time() - start_time < follow_seconds:
        frame = camera.capture_frame()
        if frame is None:
            apply_manual_override(vehicle, 1500, 1500, throttle_pwm, 1500)
            time.sleep(0.2)
            continue
        detections = detector.detect(frame)
        selected_detection = None
        if target_bbox is None:
            if detections:
                selected_detection = detections[0]
                target_bbox = selected_detection["bbox"]
                last_seen = time.time()
                logger.info("Target acquired")
        else:
            best_iou = 0.0
            for det in detections:
                iou = bbox_iou(target_bbox, det["bbox"])
                if iou > best_iou:
                    best_iou = iou
                    selected_detection = det
            if best_iou < 0.05:
                selected_detection = None

        if selected_detection is None:
            if target_bbox and (time.time() - last_seen) > loss_timeout:
                logger.warning("Target lost > %.1fs, aborting follow", loss_timeout)
                return False
            apply_manual_override(vehicle, 1500, 1500, throttle_pwm, 1500)
            time.sleep(0.2)
            continue

        target_bbox = selected_detection["bbox"]
        last_seen = time.time()

        bbox_distance = estimate_distance_from_bbox(target_bbox, frame.shape[:2])
        bottom_distance = bottom_sensor.get_distance()
        front_distance = front_sensor.get_distance()
        h, w = frame.shape[:2]
        cx = (target_bbox[0] + target_bbox[2]) / 2.0
        cy = (target_bbox[1] + target_bbox[3]) / 2.0
        error_x = abs(cx - w / 2) / w
        error_y = abs(cy - h / 2) / h
        distance = clamp(blend_distance(bbox_distance, front_distance, error_x, error_y), DISTANCE_MIN, DISTANCE_MAX)
        altitude = bottom_distance if bottom_distance is not None else TARGET_ALTITUDE
        altitude = clamp(altitude, MIN_ALTITUDE, MAX_ALTITUDE)

        command = compute_follow_command(target_bbox, frame.shape[:2], DESIRED_DISTANCE, distance, altitude, TARGET_ALTITUDE)
        # Update throttle towards maintaining altitude
        altitude_error = TARGET_ALTITUDE - altitude
        throttle_pwm += int(altitude_error * 20)
        throttle_pwm = clamp(throttle_pwm, THROTTLE_IDLE, THROTTLE_MAX)

        send_stabilize_command(vehicle, command, throttle_pwm)
        time.sleep(0.1)

    logger.info("Follow duration complete")
    return True


# ---------------------------------------------------------------------------
# CLI / main
# ---------------------------------------------------------------------------

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="STABILIZE mode follow script with YOLO tracking")
    parser.add_argument("--connect", default=CONNECTION_STRING)
    parser.add_argument("--baud", type=int, default=BAUD_RATE)
    parser.add_argument("--target-alt", type=float, default=TARGET_ALTITUDE)
    parser.add_argument("--hold-seconds", type=int, default=10)
    parser.add_argument("--follow-seconds", type=int, default=FOLLOW_DURATION)
    parser.add_argument("--model-path", default=DEFAULT_MODEL_PATH)
    parser.add_argument("--input-size", type=int, default=320)
    parser.add_argument("--target-class", default=DEFAULT_TARGET_CLASS, choices=list(COCO_CLASS_IDS.keys()))
    parser.add_argument("--confidence", type=float, default=0.5)
    parser.add_argument("--nms", type=float, default=0.4)
    parser.add_argument("--loss-timeout", type=float, default=LOSS_TIMEOUT)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--log-level", default="INFO")
    return parser.parse_args()


def configure_logging(level: str) -> None:
    logging.basicConfig(level=getattr(logging, level.upper(), logging.INFO), format="%(asctime)s [%(levelname)s] %(name)s: %(message)s")


def run_sequence(args: argparse.Namespace) -> int:
    bottom_sensor = ToFSensor(BOTTOM_SENSOR_CHANNEL, log_height=True)
    front_sensor = ToFSensor(FRONT_SENSOR_CHANNEL)
    camera = CameraManager(FRAME_WIDTH, FRAME_HEIGHT, FRAME_RATE)
    vehicle = None

    def handle_sigint(signum, frame):  # pragma: no cover
        raise KeyboardInterrupt()

    signal.signal(signal.SIGINT, handle_sigint)

    try:
        if not bottom_sensor.initialize():
            return 1
        if not front_sensor.initialize():
            return 1
        if not camera.initialize():
            return 1

        target_class_id = COCO_CLASS_IDS[args.target_class]
        detector = YOLODetector(
            model_path=args.model_path,
            input_size=args.input_size,
            confidence=args.confidence,
            nms=args.nms,
            target_class_id=target_class_id,
        )
        if detector.net is None:
            logger.error("Detector not ready")
            return 1

        if args.dry_run:
            logger.warning("Dry-run enabled; skipping flight")
            end_time = time.time() + min(10, args.follow_seconds)
            while time.time() < end_time:
                _ = bottom_sensor.get_distance()
                _ = front_sensor.get_distance()
                frame = camera.capture_frame()
                if frame is not None:
                    _ = detector.detect(frame)
                time.sleep(0.2)
            return 0

        logger.info("Connecting to %s @ %s baud", args.connect, args.baud)
        vehicle = connect(args.connect, baud=args.baud, wait_ready=True)
        logger.info("Connected to vehicle %s", vehicle.version)

        if not set_mode(vehicle, "STABILIZE"):
            return 1
        if not arm_vehicle(vehicle):
            return 1

        throttle = ramp_until_takeoff(vehicle, bottom_sensor, THROTTLE_IDLE, THROTTLE_MAX, TAKEOFF_THRESHOLD)
        if throttle is None:
            return 1

        gains = {
            "kp": 15.0,
            "ki": 2.0,
            "kd": 5.0,
            "min_pwm": THROTTLE_IDLE,
            "max_pwm": THROTTLE_MAX,
            "step_up": THROTTLE_STEP_UP,
            "step_down": THROTTLE_STEP_DOWN,
        }
        throttle = altitude_pid_loop(
            vehicle, bottom_sensor, args.target_alt, ALTITUDE_TOLERANCE, gains, throttle, duration=None, timeout=30
        )
        throttle = altitude_pid_loop(
            vehicle, bottom_sensor, args.target_alt, ALTITUDE_TOLERANCE, gains, throttle, duration=args.hold_seconds, timeout=args.hold_seconds + 10
        )

        follow_success = follow_target_loop(
            vehicle=vehicle,
            camera=camera,
            detector=detector,
            front_sensor=front_sensor,
            bottom_sensor=bottom_sensor,
            hover_throttle=throttle,
            follow_seconds=args.follow_seconds,
            loss_timeout=args.loss_timeout,
        )
        if not follow_success:
            logger.warning("Follow aborted due to target loss")

        descend_to_ground(vehicle, bottom_sensor, throttle)
        final_touchdown(vehicle, bottom_sensor)
        disarm_vehicle(vehicle)
        logger.info("Follow sequence completed")
        return 0
    except KeyboardInterrupt:
        logger.warning("Interrupted by user")
        return 1
    except Exception as exc:
        logger.error("Unexpected error: %s", exc)
        import traceback

        logger.debug(traceback.format_exc())
        return 1
    finally:
        if vehicle is not None:
            try:
                clear_overrides(vehicle)
            except Exception:
                pass
            try:
                vehicle.close()
            except Exception:
                pass
        bottom_sensor.cleanup()
        front_sensor.cleanup()
        camera.cleanup()


def main() -> int:
    args = parse_args()
    configure_logging(args.log_level)
    return run_sequence(args)


if __name__ == "__main__":
    sys.exit(main())


