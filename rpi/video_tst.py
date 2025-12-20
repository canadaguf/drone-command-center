#!/usr/bin/env python3

from picamera2 import Picamera2
from picamera2.encoders import H264Encoder
from picamera2.outputs import FfmpegOutput
import time
import datetime
import os

# Output directory (optional)
VIDEO_DIR = "/home/ilya/videos"
os.makedirs(VIDEO_DIR, exist_ok=True)

# Generate timestamped filename
timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
video_path = os.path.join(VIDEO_DIR, f"video_{timestamp}.mp4")

# Initialize camera
picam2 = Picamera2()
video_config = picam2.create_video_configuration()
picam2.configure(video_config)

# Set up encoder and output
encoder = H264Encoder(bitrate=10000000)  # 10 Mbps
output = FfmpegOutput(video_path, audio=False)

# Start recording
picam2.start_recording(encoder, output)
print(f"Recording to: {video_path}")
print("Press Ctrl+C to stop...")

try:
    # Run indefinitely until interrupted
    while True:
        time.sleep(1)

except KeyboardInterrupt:
    print("\nStopping recording...")

finally:
    picam2.stop_recording()
    print(f"✅ Video saved as: {video_path}")