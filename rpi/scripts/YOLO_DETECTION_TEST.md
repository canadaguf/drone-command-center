# YOLO Detection Test Guide

## Quick Start

Test YOLO detection with live camera feed:

```bash
cd /path/to/drone-command-center/rpi/scripts
python3 test_yolo_detection.py
```

## What It Tests

1. **YOLO Model Loading**: Verifies the ONNX model loads correctly
2. **Camera Initialization**: Tests camera capture
3. **Detection Pipeline**: Runs YOLO detection on live frames
4. **Person Tracking**: Tests the person tracker (assigns persistent IDs)
5. **Visualization**: Shows detections with bounding boxes and IDs

## Controls

- **'q'**: Quit the test
- **'s'**: Save current frame with detections to disk

## Expected Output

When running, you should see:
- A window showing the camera feed with green boxes around detected objects
- Red boxes with IDs for tracked persons
- Console output every 30 frames showing detection info
- FPS counter and detection stats

## Test Without Camera

If you just want to test YOLO model loading without camera:

```bash
python3 test_yolo_detection.py --no-camera
```

## Troubleshooting

### No Detections

1. **Check lighting**: YOLO works best with good lighting
2. **Check distance**: Person should be clearly visible (not too far)
3. **Check model path**: Verify model exists at configured path
4. **Lower confidence**: Try lowering confidence threshold in config

### Camera Not Working

1. Check camera is connected: `libcamera-hello --list-cameras`
2. Check permissions: `groups` (should include video group)
3. Test camera separately: `python3 test_snippets/simple_camera_test.py`

### Model Not Found

```bash
# Download model
python3 scripts/download_yolo_model.py

# Or manually check path
ls -lh ~/models/yolo11n.onnx
```

## Integration with Main System

Once this test works, the same components are used in `drone_client/main.py`:
- `YOLODetector` - Same YOLO detection
- `CameraManager` - Same camera interface  
- `PersonTracker` - Same tracking logic

If this test works, detections should appear in the frontend when running the full system.

