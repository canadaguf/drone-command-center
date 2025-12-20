#!/usr/bin/env python3

from picamera2 import Picamera2
from picamera2.encoders import H264Encoder
from picamera2.outputs import FfmpegOutput
import time

# Initialize camera
picam2 = Picamera2()
video_config = picam2.create_video_configuration()
picam2.configure(video_config)

# Set up encoder and output with timestamp support via FFmpeg
encoder = H264Encoder(bitrate=10000000)  # 10 Mbps; adjust if needed
output = FfmpegOutput('video_with_timestamps.mp4', audio=False)

# Start recording
picam2.start_recording(encoder, output)
print("Recording... Press Ctrl+C to stop.")

try:
    # Record for 10 seconds (or run indefinitely until interrupted)
    time.sleep(10)
except KeyboardInterrupt:
    print("Stopping recording...")

picam2.stop_recording()
print("Video saved as video_with_timestamps.mp4")