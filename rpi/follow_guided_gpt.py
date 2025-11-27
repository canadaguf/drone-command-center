#!/usr/bin/env python3
"""
Autonomous GUIDED-mode follow script.

Flow:
1. Initialize bottom (channel 1) and forward (channel 0) VL53L1X sensors via TCA9548A multiplexer
2. Connect to ArduCopter over the `/dev/ttyAMA0` serial link (256000 baud)
3. Arm, perform gentle liftoff using the tested ramp/ToF logic, climb to 1 m and stabilize
4. Start the vision pipeline (Picamera2 + YOLO ONNX + simple tracker) to lock onto one target class
5. Convert bounding-box offsets + blended ToF distance to velocity commands and follow for up to 60 s
6. If the target is lost for longer than the configured timeout, or once follow time elapses, land slowly

The script is self-contained so it can run outside of the main client without any frontend connection.
"""

import argparse
import collections
import collections.abc
import errno
import logging
import math
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
# Constants / configuration defaults
# ---------------------------------------------------------------------------

CONNECTION_STRING = "/dev/ttyAMA0"
BAUD_RATE = 256000
TARGET_ALTITUDE = 1.0
MIN_ALTITUDE = 0.5
MAX_ALTITUDE = 1.5
HOLD_DURATION = 10
FOLLOW_DURATION = 60
GROUND_THRESHOLD = 0.08
TAKEOFF_THRESHOLD = 0.1
ALTITUDE_TOLERANCE = 0.05
THROTTLE_IDLE = 1100
THROTTLE_MAX = 1900
THROTTLE_STEP = 4

FRONT_SENSOR_CHANNEL = 0
BOTTOM_SENSOR_CHANNEL = 1
MULTIPLEXER_ADDRESS = 0x70
VL53L1X_ADDRESS = 0x29
MAX_I2C_RETRIES = 5
BASE_RETRY_DELAY = 0.02
HEIGHT_LOG_INTERVAL = 1.0

DEFAULT_MODEL_PATH = "yolo11s_person.onnx"
DEFAULT_TARGET_CLASS = "chair"
COCO_CLASS_IDS = {
    "person": 0,
    "chair": 56,
    "bench": 14,
}

DESIRED_DISTANCE = 3.0
DISTANCE_MIN = 0.8
DISTANCE_MAX = 5.0
KP_DISTANCE = 0.35
KP_YAW = 0.45
KP_PITCH = 0.25
KP_ALTITUDE = 1.2

LOSS_TIMEOUT = 5.0
RECOVERY_ALTITUDE = 0.8
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
FRAME_RATE = 20

logger = logging.getLogger("follow_guided")
LAST_HEIGHT_LOG = 0.0


# ---------------------------------------------------------------------------
# Utilities
# ---------------------------------------------------------------------------

def clamp(value: float, min_value: float, max_value: float) -> float:
    return max(min_value, min(max_value, value))


# ---------------------------------------------------------------------------
# ToF sensor helpers
# ---------------------------------------------------------------------------

class ToFSensor:
    """VL53L1X helper with multiplexer channel selection and retry logic."""

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
        except Exception as exc:  # pragma: no cover - hardware specific
            logger.error("Failed to initialize ToF sensor on channel %s: %s", self.channel, exc)
            return False

    def _select_channel(self, channel: int, retry: int = 0) -> None:
        if not self.smbus:
            raise RuntimeError("SMBus not initialised")
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
# Camera helper
# ---------------------------------------------------------------------------

class CameraManager:
    """Minimal Picamera2 wrapper to fetch RGB frames."""

    def __init__(self, width: int, height: int, fps: int):
        self.width = width
        self.height = height
        self.fps = fps
        self.picam2: Optional[Picamera2] = None
        self.initialized = False

    def initialize(self) -> bool:
        if Picamera2 is None:
            logger.error("Picamera2 not available on this system")
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
            logger.info("Camera initialized at %sx%s @ %sfps", self.width, self.height, self.fps)
            return True
        except Exception as exc:
            logger.error("Failed to initialise camera: %s", exc)
            return False

    def capture_frame(self) -> Optional[np.ndarray]:
        if not self.initialized or not self.picam2:
            return None
        try:
            frame = self.picam2.capture_array()
            return frame
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


# ---------------------------------------------------------------------------
# Detection & tracking helpers
# ---------------------------------------------------------------------------

class YOLODetector:
    """Single-class YOLO ONNX detector using OpenCV DNN."""

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
        num_candidates = outputs.shape[1]

        for i in range(num_candidates):
            classes_scores = outputs[4:, i]
            class_id = int(np.argmax(classes_scores))
            confidence = classes_scores[class_id]
            if class_id != self.target_class_id:
                continue
            if confidence < self.confidence:
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
    bbox_height_ratio = (y2 - y1) / max(1, h)
    bbox_width_ratio = (x2 - x1) / max(1, w)

    # heuristics derived from calibration data similar to depth_estimator.py
    if bbox_height_ratio <= 0:
        height_distance = DESIRED_DISTANCE
    else:
        height_distance = clamp(0.9 / bbox_height_ratio, 0.5, 8.0)

    if bbox_width_ratio <= 0:
        width_distance = DESIRED_DISTANCE
    else:
        width_distance = clamp(0.6 / bbox_width_ratio, 0.5, 8.0)

    return (height_distance + width_distance) / 2.0


# ---------------------------------------------------------------------------
# DroneKit helpers (copied/adapted from liftoff_guided_gpt.py)
# ---------------------------------------------------------------------------

def wait_until_armable(vehicle, timeout: float) -> bool:
    logger.info("Waiting for vehicle to become armable...")
    start = time.time()
    while time.time() - start < timeout:
        if vehicle.is_armable:
            logger.info("Vehicle is armable")
            return True
        time.sleep(1.0)
    logger.error("Vehicle never became armable")
    return False


def ensure_gps_lock(vehicle, min_fix: int, min_sat: int) -> bool:
    gps = getattr(vehicle, "gps_0", None)
    if gps is None:
        logger.error("No GPS data available")
        return False
    logger.info("GPS fix_type=%s satellites=%s", gps.fix_type, gps.satellites_visible)
    if gps.fix_type >= min_fix and gps.satellites_visible >= min_sat:
        return True
    logger.error("Insufficient GPS lock")
    return False


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


def apply_throttle_override(vehicle, pwm: int) -> None:
    pwm = max(1000, min(2000, pwm))
    vehicle.channels.overrides = {"1": 1500, "2": 1500, "3": pwm, "4": 1500}


def clear_rc_override(vehicle) -> None:
    vehicle.channels.overrides = {}


def gradual_liftoff(vehicle, bottom_sensor: ToFSensor, idle_pwm: int, max_pwm: int, takeoff_threshold: float) -> bool:
    logger.info("Ramping throttle until ToF exceeds %.2fm", takeoff_threshold)
    throttle = idle_pwm
    last_log = time.time()
    while throttle <= max_pwm:
        apply_throttle_override(vehicle, throttle)
        time.sleep(0.15)
        distance = bottom_sensor.get_distance()
        if distance is not None and distance > takeoff_threshold:
            logger.info("Takeoff detected (%.2fm) at throttle %s", distance, throttle)
            clear_rc_override(vehicle)
            return True
        throttle += THROTTLE_STEP
        if time.time() - last_log > 1.0:
            logger.debug("Throttle %s, ToF=%.3fm", throttle, -1 if distance is None else distance)
            last_log = time.time()
    logger.error("Reached max throttle without detecting liftoff")
    clear_rc_override(vehicle)
    return False


def command_takeoff(vehicle, target_alt: float) -> bool:
    try:
        vehicle.simple_takeoff(target_alt)
        logger.info("simple_takeoff(%.2f) sent", target_alt)
        return True
    except Exception as exc:
        logger.error("simple_takeoff failed: %s", exc)
        return False


def wait_for_altitude(bottom_sensor: ToFSensor, target_alt: float, tolerance: float, timeout: float = 30.0) -> bool:
    logger.info("Holding until altitude %.2fm ± %.2fm", target_alt, tolerance)
    start = time.time()
    while time.time() - start < timeout:
        distance = bottom_sensor.get_distance()
        if distance is not None and abs(distance - target_alt) <= tolerance:
            logger.info("Altitude stable at %.2fm", distance)
            return True
        time.sleep(0.2)
    logger.warning("Altitude wait timed out")
    return False


def send_velocity_command(vehicle, vx: float, vy: float, vz: float, yaw_rate: float = 0.0) -> None:
    yaw_heading = math.radians(getattr(vehicle, "heading", 0.0))
    v_north = vx * math.cos(yaw_heading) - vy * math.sin(yaw_heading)
    v_east = vx * math.sin(yaw_heading) + vy * math.cos(yaw_heading)
    v_down = -vz
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        vehicle._master.target_system if hasattr(vehicle, "_master") else 1,
        vehicle._master.target_component if hasattr(vehicle, "_master") else 1,
        8,
        0b0000111111000111,
        0,
        0,
        0,
        v_north,
        v_east,
        v_down,
        0,
        0,
        0,
        0,
        yaw_rate,
    )
    vehicle.send_mavlink(msg)


def guided_descent(vehicle, bottom_sensor: ToFSensor, ground_threshold: float) -> None:
    logger.info("Beginning guided descent")
    start = time.time()
    while time.time() - start < 60:
        distance = bottom_sensor.get_distance()
        if distance is not None and distance <= ground_threshold * 2:
            logger.info("Near ground (%.2fm), final landing", distance)
            break
        if distance is None or distance > 0.6:
            vz = -0.35
        elif distance > 0.3:
            vz = -0.25
        else:
            vz = -0.12
        send_velocity_command(vehicle, 0.0, 0.0, vz)
        time.sleep(0.2)


def final_touchdown(vehicle, bottom_sensor: ToFSensor, ground_threshold: float) -> None:
    logger.info("Final touchdown with throttle override")
    throttle = 1400
    while True:
        distance = bottom_sensor.get_distance()
        if distance is not None and distance <= ground_threshold:
            logger.info("Touchdown detected at %.2fm", distance)
            break
        throttle = max(1100, throttle - 5)
        apply_throttle_override(vehicle, throttle)
        time.sleep(0.2)
    apply_throttle_override(vehicle, 1100)
    time.sleep(0.5)
    clear_rc_override(vehicle)


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
    vy: float
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

    altitude = altitude if altitude is not None else target_altitude
    altitude = clamp(altitude, MIN_ALTITUDE, MAX_ALTITUDE)
    altitude_error = clamp(target_altitude - altitude, -0.6, 0.6)
    vertical_bias = -KP_PITCH * error_y_norm
    vz = clamp(KP_ALTITUDE * altitude_error + vertical_bias * 0.3, -0.5, 0.5)

    return ControlCommand(vx=vx, vy=0.0, vz=vz, yaw_rate=yaw_rate)


def blend_distance(bbox_distance: float, tof_distance: Optional[float], error_x: float, error_y: float) -> float:
    if tof_distance is None:
        return bbox_distance
    if abs(error_x) < 0.1 and abs(error_y) < 0.1:
        return (bbox_distance * 0.4) + (tof_distance * 0.6)
    return bbox_distance


def follow_target_loop(
    vehicle,
    camera: CameraManager,
    detector: YOLODetector,
    front_sensor: ToFSensor,
    bottom_sensor: ToFSensor,
    follow_seconds: int,
    loss_timeout: float,
) -> bool:
    start_time = time.time()
    target_bbox: Optional[Tuple[int, int, int, int]] = None
    last_seen = 0.0

    while time.time() - start_time < follow_seconds:
        frame = camera.capture_frame()
        if frame is None:
            logger.warning("No frame from camera, holding position")
            send_velocity_command(vehicle, 0, 0, 0, 0)
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
                logger.warning("Target lost for %.1fs, aborting follow", loss_timeout)
                return False
            send_velocity_command(vehicle, 0.0, 0.0, 0.0, 0.0)
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
        distance = clamp(
            blend_distance(bbox_distance, front_distance, error_x, error_y), DISTANCE_MIN, DISTANCE_MAX
        )
        altitude = bottom_distance if bottom_distance is not None else TARGET_ALTITUDE
        target_altitude = clamp(TARGET_ALTITUDE, MIN_ALTITUDE, MAX_ALTITUDE)
        command = compute_follow_command(target_bbox, frame.shape[:2], DESIRED_DISTANCE, distance, altitude, target_altitude)
        send_velocity_command(vehicle, command.vx, command.vy, command.vz, command.yaw_rate)
        time.sleep(0.1)

    logger.info("Follow duration complete")
    return True


# ---------------------------------------------------------------------------
# Argument parsing / main control
# ---------------------------------------------------------------------------

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="GUIDED mode follow script with YOLO + ToF fusion")
    parser.add_argument("--connect", default=CONNECTION_STRING)
    parser.add_argument("--baud", type=int, default=BAUD_RATE)
    parser.add_argument("--target-alt", type=float, default=TARGET_ALTITUDE)
    parser.add_argument("--hold-seconds", type=int, default=HOLD_DURATION)
    parser.add_argument("--follow-seconds", type=int, default=FOLLOW_DURATION)
    parser.add_argument("--model-path", default=DEFAULT_MODEL_PATH)
    parser.add_argument("--input-size", type=int, default=320)
    parser.add_argument("--target-class", default=DEFAULT_TARGET_CLASS, choices=list(COCO_CLASS_IDS.keys()))
    parser.add_argument("--confidence", type=float, default=0.5)
    parser.add_argument("--nms", type=float, default=0.4)
    parser.add_argument("--loss-timeout", type=float, default=LOSS_TIMEOUT)
    parser.add_argument("--dry-run", action="store_true", help="Skip DroneKit commands (vision only)")
    parser.add_argument("--log-level", default="INFO")
    return parser.parse_args()


def configure_logging(level: str) -> None:
    logging.basicConfig(level=getattr(logging, level.upper(), logging.INFO), format="%(asctime)s [%(levelname)s] %(name)s: %(message)s")


def run_sequence(args: argparse.Namespace) -> int:
    bottom_sensor = ToFSensor(BOTTOM_SENSOR_CHANNEL, log_height=True)
    front_sensor = ToFSensor(FRONT_SENSOR_CHANNEL, log_height=False)
    vehicle = None
    camera = CameraManager(FRAME_WIDTH, FRAME_HEIGHT, FRAME_RATE)

    def handle_sigint(signum, frame):  # pragma: no cover - signal handler
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
            logger.warning("Dry-run mode enabled; skipping flight operations")
            # Exercise the detector + sensors for a short period
            end_time = time.time() + min(10, args.follow_seconds)
            while time.time() < end_time:
                _ = front_sensor.get_distance()
                _ = bottom_sensor.get_distance()
                frame = camera.capture_frame()
                if frame is not None:
                    _ = detector.detect(frame)
                time.sleep(0.2)
            return 0

        logger.info("Connecting to %s @ %s baud", args.connect, args.baud)
        vehicle = connect(args.connect, baud=args.baud, wait_ready=True)
        logger.info("Connected to vehicle %s", vehicle.version)

        if not wait_until_armable(vehicle, 30):
            return 1
        if not ensure_gps_lock(vehicle, 3, 6):
            return 1
        if not set_mode(vehicle, "GUIDED"):
            return 1
        if not arm_vehicle(vehicle):
            return 1
        if not gradual_liftoff(vehicle, bottom_sensor, THROTTLE_IDLE, THROTTLE_MAX, TAKEOFF_THRESHOLD):
            return 1
        if not command_takeoff(vehicle, args.target_alt):
            return 1
        wait_for_altitude(bottom_sensor, args.target_alt, ALTITUDE_TOLERANCE)
        time.sleep(args.hold_seconds)

        follow_success = follow_target_loop(
            vehicle=vehicle,
            camera=camera,
            detector=detector,
            front_sensor=front_sensor,
            bottom_sensor=bottom_sensor,
            follow_seconds=args.follow_seconds,
            loss_timeout=args.loss_timeout,
        )

        if not follow_success:
            logger.warning("Follow aborted due to target loss")
        guided_descent(vehicle, bottom_sensor, GROUND_THRESHOLD)
        final_touchdown(vehicle, bottom_sensor, GROUND_THRESHOLD)
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
                clear_rc_override(vehicle)
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


