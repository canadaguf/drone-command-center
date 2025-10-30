# YOLO11n Model Setup Guide

## Quick Setup

Run the download script:

```bash
cd /home/ilya/drone-command-center/rpi/scripts
python3 download_yolo_model.py
```

This will:
1. Create `~/models` directory if it doesn't exist
2. Download YOLO11n ONNX model (~6MB)
3. Place it at `~/models/yolo11n.onnx`

## Manual Setup

### Option 1: Download ONNX Directly

```bash
mkdir -p ~/models
cd ~/models
wget https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n.onnx
```

### Option 2: Download PyTorch and Convert

```bash
mkdir -p ~/models
cd ~/models

# Download PyTorch model
wget https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n.pt

# Convert to ONNX (requires ultralytics)
python3 << EOF
from ultralytics import YOLO
model = YOLO('yolo11n.pt')
model.export(format='onnx', imgsz=320, simplify=True)
EOF

# Verify
ls -lh yolo11n.onnx
```

## Verify Installation

Check if model exists:

```bash
ls -lh ~/models/yolo11n.onnx
```

Expected size: ~6-7 MB

## Configuration

The model path is configured in `drone_client/config/production.yaml`:

```yaml
vision:
  model_path: "/home/ilya/models/yolo11n.onnx"  # or "/home/pi/models/yolo11n.onnx"
  input_size: 320
  confidence: 0.5
```

Update the path if you installed it elsewhere.

## Testing

Once the model is installed, restart the drone client:

```bash
python3 -m drone_client.main
```

You should see:
- "YOLO model loaded successfully" (instead of warnings)
- Vision loop will be active
- Person detection will work

## Troubleshooting

### Model Not Found

Error: `Failed to load YOLO model: [Errno 2] No such file or directory`

**Fix:**
1. Check model path in config
2. Verify file exists: `ls -l /path/to/yolo11n.onnx`
3. Update config with correct path

### Model Format Error

Error: `Failed to load YOLO model: Invalid model format`

**Fix:**
- Make sure you're using ONNX format (.onnx extension)
- Re-download using the script
- Verify file integrity: `file yolo11n.onnx` should show it's a valid file

### Model Too Large

If disk space is an issue:
- YOLO11n is the smallest model (~6MB)
- YOLO11s/m/l/x are larger but more accurate
- You can remove the .pt file after conversion to save space

## Model Variants

If you want better accuracy (but larger size):

- **yolo11n.onnx** - Nano (smallest, fastest) - ~6MB ✓ Recommended
- **yolo11s.onnx** - Small - ~22MB
- **yolo11m.onnx** - Medium - ~52MB
- **yolo11l.onnx** - Large - ~104MB
- **yolo11x.onnx** - XLarge (best accuracy) - ~170MB

Update the download script URL if you want a different variant.

## Next Steps

Once model is installed:
1. ✅ Restart drone client
2. ✅ Verify vision loop is active
3. ✅ Test person detection
4. ✅ Test tracking functionality

