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
from ultralytics import YOLO
from picamera2 import Picamera2

CURRENT_DIR = Path(__file__).parent
if str(CURRENT_DIR) not in sys.path:
    sys.path.insert(0, str(CURRENT_DIR))

from tof_reader import ToFReader
from sbus_reader import SBUSReader


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


def parse_detections(result) -> List[Dict[str, Any]]:
    detections: List[Dict[str, Any]] = []
    if result is None or result.boxes is None:
        return detections
    names = result.names
    for idx, box in enumerate(result.boxes):
        xyxy = box.xyxy[0].tolist()
        cls_id = int(box.cls[0])
        cls_name = names.get(cls_id, str(cls_id)) if isinstance(names, dict) else str(cls_id)
        conf = float(box.conf[0]) if box.conf is not None else 0.0
        detections.append(
            {
                "id": idx + 1,
                "bbox": (int(xyxy[0]), int(xyxy[1]), int(xyxy[2]), int(xyxy[3])),
                "cls": cls_name,
                "conf": conf,
            }
        )
    return detections


def draw_detections(frame: np.ndarray, detections: List[Dict[str, Any]]) -> None:
    for det in detections:
        x1, y1, x2, y2 = det["bbox"]
        label = f"{det['cls']} #{det['id']} {det['conf']:.2f}"
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
        cv2.rectangle(frame, (x1, y1 - th - 6), (x1 + tw, y1), (0, 255, 0), -1)
        cv2.putText(frame, label, (x1, y1 - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2, cv2.LINE_AA)


def open_picam(width: int, height: int, fps: int) -> Picamera2:
    picam2 = Picamera2()
    cfg = picam2.create_preview_configuration(
        main={"size": (width, height), "format": "RGB888"},
        controls={"FrameRate": fps},
    )
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

    yolo_cfg = cfg.get("yolo", {})
    model = YOLO(yolo_cfg.get("model_path", "yolo11n.pt"))

    tof_cfg = cfg.get("tof", {})
    tof_reader = ToFReader(
        bus=tof_cfg.get("bus", 1),
        address=tof_cfg.get("address", 0x29),
        poll_hz=tof_cfg.get("poll_hz", 20),
    ) if tof_cfg.get("enabled", True) else None

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

    picam2 = open_picam(width, height, fps)
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

    if tof_reader:
        tof_reader.start()
    if sbus_reader:
        sbus_reader.start()

    log_file = None
    log_writer = None
    if log_enabled:
        log_file = open(log_path, "w", newline="", encoding="utf-8")
        log_writer = csv.writer(log_file)
        log_writer.writerow(
            ["timestamp", "tof_m", "throttle", "yaw", "pitch", "roll"]
        )

    last_time = time.time()
    while not stop_flag:
        frame_rgb = picam2.capture_array()
        if frame_rgb is None:
            continue
        frame_rgb = rotate_frame(frame_rgb, rotation)
        frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)

        results = model(
            frame_rgb,
            verbose=False,
            conf=yolo_cfg.get("confidence", 0.4),
            iou=yolo_cfg.get("iou", 0.45),
            device=yolo_cfg.get("device", "cpu"),
            max_det=yolo_cfg.get("max_det", 10),
        )
        detections = parse_detections(results[0] if results else None)
        draw_detections(frame_bgr, detections)

        # Build overlay blocks
        tof_lines: List[str] = []
        if tof_reader:
            tof_val = tof_reader.latest_distance_m()
            tof_lines.append(format_value(tof_cfg.get("label", "ToF (m)"), tof_val))

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
                    tof_reader.latest_distance_m() if tof_reader else None,
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
    if tof_reader:
        tof_reader.stop()
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

