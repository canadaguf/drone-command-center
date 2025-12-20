#!/usr/bin/env python3
"""
Video recorder with ToF sensor overlays.
Uses video_tst.py recording approach for proper timestamps.
Adds ToF sensor readings as overlay (forward and down sensors).
"""

from picamera2 import Picamera2
from picamera2.encoders import H264Encoder
from picamera2.outputs import FfmpegOutput
import cv2
import numpy as np
import time
import errno
import logging
import argparse
import threading
from datetime import datetime
from pathlib import Path
import board
import busio
import adafruit_vl53l1x
from smbus2 import SMBus

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# Multiplexer configuration (from tof_test_multiplexer.py)
MULTIPLEXER_ADDRESS = 0x70
CHANNEL_FORWARD = 0
CHANNEL_DOWN = 1
VL53L1X_ADDRESS = 0x29
MAX_I2C_RETRIES = 5
BASE_RETRY_DELAY = 0.02


def select_multiplexer_channel(bus, channel, retry_count=0):
    """Select multiplexer channel with retry logic."""
    try:
        channel_mask = 1 << channel
        bus.write_byte(MULTIPLEXER_ADDRESS, channel_mask)
        time.sleep(0.01)
    except OSError as e:
        if (e.errno == errno.EAGAIN or e.errno == 11) and retry_count < MAX_I2C_RETRIES:
            delay = BASE_RETRY_DELAY * (2 ** retry_count)
            time.sleep(delay)
            return select_multiplexer_channel(bus, channel, retry_count + 1)
        else:
            raise


class ToFSensorReader:
    """ToF sensor reader using multiplexer (from tof_test_multiplexer.py)."""
    
    def __init__(self):
        self.smbus = None
        self.i2c = None
        self.sensor_forward = None
        self.sensor_down = None
        self.initialized = False
        self.forward_distance = None
        self.down_distance = None
        self.lock = threading.Lock()
    
    def initialize(self) -> bool:
        """Initialize ToF sensors."""
        try:
            logger.info("Initializing ToF sensors...")
            self.smbus = SMBus(1)
            self.i2c = busio.I2C(board.SCL, board.SDA, frequency=100000)
            
            # Initialize forward sensor
            try:
                select_multiplexer_channel(self.smbus, CHANNEL_FORWARD)
                time.sleep(0.1)
                self.sensor_forward = adafruit_vl53l1x.VL53L1X(self.i2c, address=VL53L1X_ADDRESS)
                self.sensor_forward.start_ranging()
                logger.info("✓ Forward ToF sensor initialized")
            except Exception as e:
                logger.warning(f"Failed to initialize forward sensor: {e}")
                self.sensor_forward = None
            
            # Initialize down sensor
            try:
                select_multiplexer_channel(self.smbus, CHANNEL_DOWN)
                time.sleep(0.1)
                self.sensor_down = adafruit_vl53l1x.VL53L1X(self.i2c, address=VL53L1X_ADDRESS)
                self.sensor_down.start_ranging()
                logger.info("✓ Down ToF sensor initialized")
            except Exception as e:
                logger.warning(f"Failed to initialize down sensor: {e}")
                self.sensor_down = None
            
            if self.sensor_forward is None and self.sensor_down is None:
                logger.error("No ToF sensors initialized")
                return False
            
            self.initialized = True
            return True
        except Exception as e:
            logger.error(f"Failed to initialize ToF sensors: {e}")
            return False
    
    def read_forward(self) -> float:
        """Read forward sensor distance in meters."""
        if not self.initialized or self.sensor_forward is None:
            return None
        try:
            select_multiplexer_channel(self.smbus, CHANNEL_FORWARD)
            time.sleep(0.02)
            
            for attempt in range(3):
                try:
                    if self.sensor_forward.data_ready:
                        distance_cm = self.sensor_forward.distance
                        self.sensor_forward.clear_interrupt()
                        return distance_cm / 100.0  # Convert to meters
                    break
                except OSError as e:
                    if e.errno == errno.EAGAIN or e.errno == 11:
                        time.sleep(0.01 * (attempt + 1))
                        continue
                    else:
                        raise
        except Exception as e:
            logger.debug(f"Error reading forward sensor: {e}")
        return None
    
    def read_down(self) -> float:
        """Read down sensor distance in meters."""
        if not self.initialized or self.sensor_down is None:
            return None
        try:
            select_multiplexer_channel(self.smbus, CHANNEL_DOWN)
            time.sleep(0.02)
            
            for attempt in range(3):
                try:
                    if self.sensor_down.data_ready:
                        distance_cm = self.sensor_down.distance
                        self.sensor_down.clear_interrupt()
                        return distance_cm / 100.0  # Convert to meters
                    break
                except OSError as e:
                    if e.errno == errno.EAGAIN or e.errno == 11:
                        time.sleep(0.01 * (attempt + 1))
                        continue
                    else:
                        raise
        except Exception as e:
            logger.debug(f"Error reading down sensor: {e}")
        return None
    
    def update_readings(self):
        """Update sensor readings (called from thread)."""
        with self.lock:
            self.forward_distance = self.read_forward()
            self.down_distance = self.read_down()
    
    def get_readings(self):
        """Get current readings (thread-safe)."""
        with self.lock:
            return self.forward_distance, self.down_distance
    
    def cleanup(self):
        """Cleanup sensors."""
        if self.sensor_forward:
            try:
                select_multiplexer_channel(self.smbus, CHANNEL_FORWARD)
                self.sensor_forward.stop_ranging()
            except:
                pass
        if self.sensor_down:
            try:
                select_multiplexer_channel(self.smbus, CHANNEL_DOWN)
                self.sensor_down.stop_ranging()
            except:
                pass
        if self.smbus:
            try:
                self.smbus.close()
            except:
                pass


def create_tof_overlay(width, height, forward_dist, down_dist):
    """Create overlay image with ToF readings."""
    # Picamera2 overlay needs RGBA format: (height, width, 4) with uint8
    overlay = np.zeros((height, width, 4), dtype=np.uint8)
    
    # Text settings
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.8
    thickness = 2
    
    # Draw forward ToF reading (top right)
    if forward_dist is not None:
        text_forward = f"Forward ToF: {forward_dist:.2f}m"
    else:
        text_forward = "Forward ToF: N/A"
    
    text_size_forward, _ = cv2.getTextSize(text_forward, font, font_scale, thickness)
    text_x_forward = width - text_size_forward[0] - 20
    text_y_forward = 35
    
    # Background rectangle for forward reading (BGR + Alpha)
    cv2.rectangle(overlay, 
                 (text_x_forward - 5, text_y_forward - text_size_forward[1] - 5),
                 (text_x_forward + text_size_forward[0] + 5, text_y_forward + 5),
                 (0, 0, 0, 220), -1)  # Black background, mostly opaque
    
    # Draw text (BGR + Alpha) - Green text, fully opaque
    cv2.putText(overlay, text_forward, (text_x_forward, text_y_forward),
               font, font_scale, (0, 255, 0, 255), thickness)
    
    # Draw down ToF reading (below forward)
    if down_dist is not None:
        text_down = f"Down ToF: {down_dist:.2f}m"
    else:
        text_down = "Down ToF: N/A"
    
    text_size_down, _ = cv2.getTextSize(text_down, font, font_scale, thickness)
    text_x_down = width - text_size_down[0] - 20
    text_y_down = text_y_forward + text_size_forward[1] + 25
    
    # Background rectangle for down reading
    cv2.rectangle(overlay,
                 (text_x_down - 5, text_y_down - text_size_down[1] - 5),
                 (text_x_down + text_size_down[0] + 5, text_y_down + 5),
                 (0, 0, 0, 220), -1)  # Black background, mostly opaque
    
    # Draw text - Green text, fully opaque
    cv2.putText(overlay, text_down, (text_x_down, text_y_down),
               font, font_scale, (0, 255, 0, 255), thickness)
    
    return overlay


def main():
    parser = argparse.ArgumentParser(description="Record video with ToF sensor overlays")
    parser.add_argument("--output", default=None,
                       help="Output video path (default: timestamped)")
    
    args = parser.parse_args()
    
    # Generate output filename if not provided
    if args.output is None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_path = f"/home/ilya/videos/tof_recording_{timestamp}.mp4"
    else:
        output_path = args.output
    
    # Create output directory if needed
    Path(output_path).parent.mkdir(parents=True, exist_ok=True)
    
    logger.info(f"Starting video recording with ToF overlays")
    logger.info(f"Output: {output_path}")
    logger.info("Press Ctrl+C to stop")
    
    # Initialize camera
    # Note: Overlays in Picamera2 work with preview streams
    # We'll use video config but ensure overlay is set correctly
    picam2 = Picamera2()
    video_config = picam2.create_video_configuration()
    picam2.configure(video_config)
    
    # Get camera dimensions for overlay
    camera_config = picam2.camera_config
    width = camera_config['main']['size'][0]
    height = camera_config['main']['size'][1]
    
    # Initialize ToF sensors
    tof_reader = ToFSensorReader()
    if not tof_reader.initialize():
        logger.warning("ToF sensors not available - recording without ToF overlays")
        tof_reader = None
    
    # Set up encoder and output with timestamp support via FFmpeg (same as video_tst.py)
    encoder = H264Encoder(bitrate=10000000)  # 10 Mbps
    output = FfmpegOutput(output_path, audio=False)
    
    # Start camera
    picam2.start()
    time.sleep(2)  # Wait for camera to stabilize
    
    # Set initial overlay before recording (empty overlay)
    # Overlay must match camera resolution exactly
    initial_overlay = np.zeros((height, width, 4), dtype=np.uint8)
    try:
        picam2.set_overlay(initial_overlay)
        logger.info(f"Overlay system initialized ({width}x{height})")
    except Exception as e:
        logger.warning(f"Could not set initial overlay: {e}")
        logger.warning("Overlays may not be supported with this camera configuration")
    
    # Start recording (same as video_tst.py) - this handles timestamps properly
    picam2.start_recording(encoder, output)
    logger.info("Recording started...")
    
    # Thread to update ToF readings and overlay
    recording = True
    
    def overlay_update_loop():
        """Update overlay with ToF readings while recording."""
        update_count = 0
        error_count = 0
        while recording:
            if tof_reader:
                tof_reader.update_readings()
                forward_dist, down_dist = tof_reader.get_readings()
                
                # Create overlay
                overlay_img = create_tof_overlay(width, height, forward_dist, down_dist)
                
                # Verify overlay format
                if overlay_img.shape != (height, width, 4):
                    logger.error(f"Overlay shape mismatch: {overlay_img.shape}, expected ({height}, {width}, 4)")
                
                # Set overlay on camera
                try:
                    picam2.set_overlay(overlay_img)
                    update_count += 1
                    error_count = 0  # Reset error count on success
                    if update_count % 10 == 0:
                        logger.info(f"Overlay updated {update_count} times - Forward: {forward_dist:.2f}m, Down: {down_dist:.2f}m")
                except Exception as e:
                    error_count += 1
                    if error_count <= 3:  # Log first few errors
                        logger.error(f"Overlay update error ({error_count}): {e}")
                    elif error_count == 4:
                        logger.error("Overlay errors continuing - check if overlays are supported during recording")
            else:
                # No ToF reader, set empty overlay
                try:
                    empty_overlay = np.zeros((height, width, 4), dtype=np.uint8)
                    picam2.set_overlay(empty_overlay)
                except Exception as e:
                    logger.debug(f"Empty overlay error: {e}")
            
            time.sleep(0.1)  # Update overlay at 10 Hz
    
    overlay_thread = threading.Thread(target=overlay_update_loop, daemon=True)
    overlay_thread.start()
    
    try:
        # Record indefinitely until Ctrl+C (same as video_tst.py but without time limit)
        while True:
            time.sleep(1)
    
    except KeyboardInterrupt:
        logger.info("Stopping recording...")
    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
    finally:
        recording = False
        picam2.stop_recording()
        picam2.stop()
        picam2.close()
        
        if tof_reader:
            tof_reader.cleanup()
        
        logger.info(f"Recording saved to: {output_path}")


if __name__ == "__main__":
    main()

