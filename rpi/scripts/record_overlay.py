"""
Standalone recorder with YOLO + ToF + RC overlays for Raspberry Pi.
"""

import argparse
import csv
import signal
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np
import yaml
import onnxruntime as ort
from picamera2 import Picamera2

CURRENT_DIR = Path(__file__).parent
if str(CURRENT_DIR) not in sys.path:
    sys.path.insert(0, str(CURRENT_DIR))

from tof_reader import ToFReader
from sbus_reader import SBUSReader

# COCO class names (80)
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


def load_config(path: Path) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def ensure_parent(path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)


def rotate_frame(frame: np.ndarray, rotation: int) -> np.ndarray:
    if rotation == 90:
        return cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
    if rotation == 180:
        return cv2.rotate(frame, cv2.ROTATE_180)
    if rotation == 270:
        return cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
    return frame


def format_value(name: str, value: Optional[float]) -> str:
    return f"{name}: {value:.2f}" if value is not None else f"{name}: --"


def draw_block(
    frame: np.ndarray,
    lines: List[str],
    position: str,
    font_scale: float,
    thickness: int,
    color: Tuple[int, int, int],
    bg_color: Tuple[int, int, int],
    bg_alpha: float,
    margin: int,
    line_spacing: int,
) -> None:
    if not lines:
        return
    font = cv2.FONT_HERSHEY_SIMPLEX
    text_sizes = [cv2.getTextSize(line, font, font_scale, thickness)[0] for line in lines]
    width = max(w for w, _ in text_sizes)
    height = sum(h for _, h in text_sizes) + line_spacing * (len(lines) - 1)
    h_pad = int(0.6 * margin)
    w_pad = int(0.6 * margin)

    if position == "top_left":
        x, y = margin, margin
    elif position == "top_right":
        x, y = frame.shape[1] - width - margin - 2 * w_pad, margin
    elif position == "bottom_left":
        x, y = margin, frame.shape[0] - height - margin - 2 * h_pad
    else:  # bottom_right
        x, y = frame.shape[1] - width - margin - 2 * w_pad, frame.shape[0] - height - margin - 2 * h_pad

    bg_x2 = x + width + 2 * w_pad
    bg_y2 = y + height + 2 * h_pad

    overlay = frame.copy()
    cv2.rectangle(overlay, (x, y), (bg_x2, bg_y2), bg_color, -1)
    cv2.addWeighted(overlay, bg_alpha, frame, 1 - bg_alpha, 0, dst=frame)

    cursor_y = y + h_pad + text_sizes[0][1]
    for line, (w, h) in zip(lines, text_sizes):
        cv2.putText(frame, line, (x + w_pad, cursor_y), font, font_scale, color, thickness, cv2.LINE_AA)
        cursor_y += h + line_spacing


def parse_onnx_detections(
    outputs: np.ndarray,
    img_shape: Tuple[int, int],
    conf_thres: float,
    iou_thres: float,
    max_det: int,
    class_names: List[str],
) -> List[Dict[str, Any]]:
    """Parse YOLOv8/11 ONNX outputs (shape [1, 84, N])."""
    detections: List[Dict[str, Any]] = []
    if outputs is None or len(outputs) == 0:
        return detections

    preds = outputs[0]  # [84, N]
    num_candidates = preds.shape[1]
    x_factor = img_shape[1] / 640.0
    y_factor = img_shape[0] / 640.0

    boxes: List[List[int]] = []
    scores: List[float] = []
    class_ids: List[int] = []

    for i in range(num_candidates):
        cls_scores = preds[4:, i]
        max_score = float(np.max(cls_scores))
        if max_score < conf_thres:
            continue
        cls_id = int(np.argmax(cls_scores))

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
        scores.append(max_score)
        class_ids.append(cls_id)

    dets: List[Dict[str, Any]] = []
    if boxes:
        indices = cv2.dnn.NMSBoxes(boxes, scores, conf_thres, iou_thres)
        if len(indices) > 0:
            indices = np.array(indices).flatten().tolist()
            for idx, i in enumerate(indices[:max_det]):
                x, y, w, h = boxes[i]
                cls_id = class_ids[i]
                cls_name = class_names[cls_id] if 0 <= cls_id < len(class_names) else str(cls_id)
                dets.append(
                    {
                        "id": idx + 1,
                        "bbox": (x, y, x + w, y + h),
                        "cls": cls_name,
                        "conf": scores[i],
                    }
                )
    return dets


def draw_detections(frame: np.ndarray, detections: List[Dict[str, Any]]) -> None:
    for det in detections:
        x1, y1, x2, y2 = det["bbox"]
        label = f"{det['cls']} #{det['id']} {det['conf']:.2f}"
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
        cv2.rectangle(frame, (x1, y1 - th - 6), (x1 + tw, y1), (0, 255, 0), -1)
        cv2.putText(frame, label, (x1, y1 - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2, cv2.LINE_AA)


def create_ort_session(model_path: str) -> Tuple[ort.InferenceSession, str]:
    """Create ONNX Runtime session and return session + input name."""
    session = ort.InferenceSession(
        model_path,
        providers=["CPUExecutionProvider"],
    )
    input_name = session.get_inputs()[0].name
    return session, input_name


def run_ort_inference(
    frame_rgb: np.ndarray,
    session: ort.InferenceSession,
    input_name: str,
    input_size: int,
    conf: float,
    iou: float,
    max_det: int,
    class_names: List[str],
) -> List[Dict[str, Any]]:
    resized = cv2.resize(frame_rgb, (input_size, input_size), interpolation=cv2.INTER_LINEAR)
    blob = resized.astype(np.float32) / 255.0
    blob = np.transpose(blob, (2, 0, 1))  # HWC -> CHW
    blob = np.expand_dims(blob, 0)  # add batch
    outputs = session.run(None, {input_name: blob})
    return parse_onnx_detections(outputs[0], frame_rgb.shape[:2], conf, iou, max_det, class_names)


AWB_MODE_MAP = {
    "auto": 0,
    "incandescent": 1,
    "tungsten": 1,
    "fluorescent": 2,
    "indoor": 2,
    "daylight": 3,
    "sunny": 3,
    "cloudy": 4,
}


def _control_value(name: str, value: Any, cast):
    try:
        return cast(value)
    except Exception:
        return None


def open_picam(width: int, height: int, fps: int, cam_controls: Dict[str, Any]) -> Picamera2:
    picam2 = Picamera2()
    awb_mode_cfg = cam_controls.get("awb_mode", "auto")
    awb_mode_val = AWB_MODE_MAP.get(str(awb_mode_cfg).lower(), 0)
    colour_temperature = cam_controls.get("colour_temperature")
    awb_gains = cam_controls.get("awb_gains")
    cfg = picam2.create_preview_configuration(
        main={"size": (width, height), "format": "RGB888"},
        controls={
            "FrameRate": _control_value("fps", fps, int),
            "AwbEnable": bool(cam_controls.get("awb_enable", True)),
            "AwbMode": awb_mode_val,
            "Brightness": _control_value("brightness", cam_controls.get("brightness", 0.0), float),
            "Contrast": _control_value("contrast", cam_controls.get("contrast", 1.0), float),
            "Saturation": _control_value("saturation", cam_controls.get("saturation", 1.0), float),
            "Sharpness": _control_value("sharpness", cam_controls.get("sharpness", 1.0), float),
            "ColourTemperature": _control_value("colour_temperature", colour_temperature, int)
            if colour_temperature is not None
            else None,
        },
    )
    if awb_gains and isinstance(awb_gains, (list, tuple)) and len(awb_gains) == 2:
        # Manual AWB gains override if provided
        cfg["controls"]["AwbEnable"] = False
        cfg["controls"]["AwbMode"] = 0
        cfg["controls"]["ColourGains"] = tuple(float(g) for g in awb_gains)
    picam2.configure(cfg)
    picam2.start()
    time.sleep(1.5)  # warm-up
    return picam2


def main(cfg_path: Path) -> None:
    cfg = load_config(cfg_path)

    camera_cfg = cfg.get("camera", {})
    width = int(camera_cfg.get("width", 1280))
    height = int(camera_cfg.get("height", 720))
    fps = int(camera_cfg.get("fps", 30))
    rotation = int(camera_cfg.get("rotation", 0))
    cam_controls = {
        "awb_enable": camera_cfg.get("awb_enable", True),
        "awb_mode": camera_cfg.get("awb_mode", "auto"),
        "brightness": camera_cfg.get("brightness", 0.0),
        "contrast": camera_cfg.get("contrast", 1.0),
        "saturation": camera_cfg.get("saturation", 1.0),
        "sharpness": camera_cfg.get("sharpness", 1.0),
    }

    yolo_cfg = cfg.get("yolo", {})
    input_size = int(yolo_cfg.get("input_size", 320))
    class_names = COCO_NAMES
    session, input_name = create_ort_session(yolo_cfg.get("model_path", "yolo11n.onnx"))

    tof_cfg = cfg.get("tof", {})
    tof_readers: Dict[str, ToFReader] = {}
    if tof_cfg.get("enabled", True):
        sensors_cfg = tof_cfg.get("sensors")
        if sensors_cfg:
            for name, scfg in sensors_cfg.items():
                tof_readers[name] = ToFReader(
                    bus=tof_cfg.get("bus", 1),
                    address=tof_cfg.get("address", 0x29),
                    poll_hz=tof_cfg.get("poll_hz", 20),
                    multiplexer_address=tof_cfg.get("multiplexer_address"),
                    channel=scfg.get("channel"),
                )
        else:
            tof_readers["tof"] = ToFReader(
                bus=tof_cfg.get("bus", 1),
                address=tof_cfg.get("address", 0x29),
                poll_hz=tof_cfg.get("poll_hz", 20),
                multiplexer_address=tof_cfg.get("multiplexer_address"),
                channel=tof_cfg.get("channel"),
            )

    sbus_cfg = cfg.get("sbus", {})
    sbus_reader = SBUSReader(
        port=sbus_cfg.get("port", "/dev/ttyS0"),
        baudrate=sbus_cfg.get("baudrate", 100000),
        inverted=sbus_cfg.get("inverted", True),
        poll_hz=sbus_cfg.get("poll_hz", 50),
    ) if sbus_cfg.get("enabled", True) else None

    overlay_cfg = cfg.get("overlay", {})
    font_scale = float(overlay_cfg.get("font_scale", 0.6))
    thickness = int(overlay_cfg.get("thickness", 2))
    color = tuple(int(v) for v in overlay_cfg.get("color", [0, 255, 0]))
    bg_color = tuple(int(v) for v in overlay_cfg.get("bg_color", [0, 0, 0]))
    bg_alpha = float(overlay_cfg.get("bg_alpha", 0.45))
    margin = int(overlay_cfg.get("margin", 10))
    line_spacing = int(overlay_cfg.get("line_spacing", 8))
    blocks = overlay_cfg.get("blocks", {})

    output_cfg = cfg.get("output", {})
    out_path = Path(output_cfg.get("video_path", "recording.mp4"))
    fourcc = cv2.VideoWriter_fourcc(*output_cfg.get("fourcc", "mp4v"))
    bitrate = int(output_cfg.get("bitrate", 8_000_000))
    log_path = Path(output_cfg.get("log_path", "recording_log.csv"))
    log_enabled = bool(output_cfg.get("log_enabled", False))

    display_preview = bool(cfg.get("general", {}).get("display_preview", False))
    exit_hotkey = cfg.get("general", {}).get("exit_hotkey", "q")

    ensure_parent(out_path)
    ensure_parent(log_path)

    picam2 = open_picam(width, height, fps, cam_controls)
    writer = cv2.VideoWriter(
        str(out_path),
        fourcc,
        fps,
        (width, height),
    )

    stop_flag = False

    def handle_sigint(signum, frame):
        nonlocal stop_flag
        stop_flag = True

    signal.signal(signal.SIGINT, handle_sigint)

    for reader in tof_readers.values():
        reader.start()
    if sbus_reader:
        sbus_reader.start()

    log_file = None
    log_writer = None
    if log_enabled:
        log_file = open(log_path, "w", newline="", encoding="utf-8")
        log_writer = csv.writer(log_file)
        log_writer.writerow(
            ["timestamp", "tof_front_m", "tof_bottom_m", "throttle", "yaw", "pitch", "roll"]
        )

    last_time = time.time()
    while not stop_flag:
        frame_rgb = picam2.capture_array()
        if frame_rgb is None:
            continue
        frame_rgb = rotate_frame(frame_rgb, rotation)
        frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)

        detections = run_ort_inference(
            frame_rgb,
            session,
            input_name,
            input_size=input_size,
            conf=float(yolo_cfg.get("confidence", 0.4)),
            iou=float(yolo_cfg.get("iou", 0.45)),
            max_det=int(yolo_cfg.get("max_det", 10)),
            class_names=class_names,
        )
        draw_detections(frame_bgr, detections)

        # Build overlay blocks
        tof_lines: List[str] = []
        front_val = None
        bottom_val = None
        if tof_readers:
            if "front" in tof_readers:
                front_val = tof_readers["front"].latest_distance_m()
                tof_lines.append(format_value(tof_cfg.get("front_label", "Front ToF (m)"), front_val))
            if "bottom" in tof_readers:
                bottom_val = tof_readers["bottom"].latest_distance_m()
                tof_lines.append(format_value(tof_cfg.get("bottom_label", "Bottom ToF (m)"), bottom_val))
            if not tof_lines:
                # fallback first reader
                only_val = next(iter(tof_readers.values())).latest_distance_m()
                tof_lines.append(format_value(tof_cfg.get("label", "ToF (m)"), only_val))

        rc_lines: List[str] = []
        if sbus_reader:
            rc = sbus_reader.map_named_channels(sbus_cfg.get("channel_map", {}))
            for name in ("throttle", "yaw", "pitch", "roll"):
                if name in rc:
                    rc_lines.append(format_value(name, rc[name]))

        det_lines = [f"{len(detections)} detections"]
        status_lines: List[str] = []

        now = time.time()
        fps_now = 1.0 / max(1e-6, now - last_time)
        last_time = now
        status_lines.append(f"FPS: {fps_now:.1f}")
        status_lines.append(time.strftime("%H:%M:%S", time.localtime(now)))

        draw_block(
            frame_bgr,
            det_lines,
            blocks.get("detections", "top_left"),
            font_scale,
            thickness,
            color,
            bg_color,
            bg_alpha,
            margin,
            line_spacing,
        )
        draw_block(
            frame_bgr,
            tof_lines,
            blocks.get("tof", "top_right"),
            font_scale,
            thickness,
            color,
            bg_color,
            bg_alpha,
            margin,
            line_spacing,
        )
        draw_block(
            frame_bgr,
            rc_lines,
            blocks.get("rc", "bottom_right"),
            font_scale,
            thickness,
            color,
            bg_color,
            bg_alpha,
            margin,
            line_spacing,
        )
        draw_block(
            frame_bgr,
            status_lines,
            blocks.get("status", "bottom_left"),
            font_scale,
            thickness,
            color,
            bg_color,
            bg_alpha,
            margin,
            line_spacing,
        )

        writer.write(frame_bgr)

        if log_writer:
            rc = sbus_reader.map_named_channels(sbus_cfg.get("channel_map", {})) if sbus_reader else {}
            log_writer.writerow(
                [
                    now,
                    front_val,
                    bottom_val,
                    rc.get("throttle"),
                    rc.get("yaw"),
                    rc.get("pitch"),
                    rc.get("roll"),
                ]
            )

        if display_preview:
            cv2.imshow("overlay", frame_bgr)
            if cv2.waitKey(1) & 0xFF == ord(exit_hotkey):
                stop_flag = True

    if log_file:
        log_file.close()
    writer.release()
    picam2.stop()
    picam2.close()
    if display_preview:
        cv2.destroyAllWindows()
    for reader in tof_readers.values():
        reader.stop()
    if sbus_reader:
        sbus_reader.stop()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Record Pi camera with overlays.")
    parser.add_argument(
        "--config",
        type=Path,
        default=Path("rpi/config/overlay_recorder.yaml"),
        help="Path to YAML config file",
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    main(args.config)

