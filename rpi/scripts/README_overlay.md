# Overlay Recorder (YOLO + ToF + RC)

## Prereqs
- Raspberry Pi with camera supported by Picamera2.
- VL53L1X on I2C bus (default bus 1, addr 0x29).
- SBUS receiver wired to `/dev/ttyS0` (100000 8E2, hardware inverter recommended).
- Python deps: `pip install -r rpi/requirements.txt` (needs `pyserial`, `ultralytics`, `opencv-python`, `picamera2`, `adafruit-circuitpython-vl53l1x`).

## Run
```bash
python rpi/scripts/record_overlay.py --config rpi/config/overlay_recorder.yaml
```

## Expected behavior
- Writes MP4 to `output.video_path`, with YOLO boxes + class/id, ToF distance, and throttle/yaw/pitch/roll overlays.
- Optional CSV log (`output.log_enabled`) with timestamp, ToF, and RC channels.
- Graceful stop on Ctrl+C; if `display_preview=true`, closes on hotkey (default `q`).

## Quick sanity checks
1) Camera: confirm live video in the MP4 or preview; adjust `camera.width/height/fps`.
2) YOLO: ensure model path exists; detections count shown in top-left block.
3) ToF: cover/uncover sensor; overlay value should change (meters).
4) RC: move sticks; normalized values (0..1) should respond.
5) If SBUS idle/absent: RC overlay stays `--`; verify wiring/baud/inversion.

