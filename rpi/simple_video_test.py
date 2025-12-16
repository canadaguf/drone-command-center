#!/usr/bin/env python3
"""
Super-simple video recording test script.
Records video with original real-world speed and real-world colors.
"""

import cv2
import time
import logging
from datetime import datetime
from pathlib import Path
from picamera2 import Picamera2
import subprocess

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


def main():
    # Default settings
    width = 1920
    height = 1080
    fps = 30
    
    # Generate output filename
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_path = f"/home/ilya/videos/test_recording_{timestamp}.mp4"
    
    # Create output directory if needed
    Path(output_path).parent.mkdir(parents=True, exist_ok=True)
    
    logger.info(f"Starting simple video recording test")
    logger.info(f"Output: {output_path}")
    logger.info(f"Resolution: {width}x{height} @ {fps}fps")
    logger.info("Press Ctrl+C to stop")
    
    camera = None
    ffmpeg_process = None
    
    try:
        # Initialize camera
        logger.info("Initializing camera...")
        camera = Picamera2()
        
        # Configuration for natural colors - try video configuration for better color accuracy
        # Use video configuration which is optimized for recording
        config = camera.create_video_configuration(
            main={"size": (width, height), "format": "RGB888"},
            controls={
                "FrameRate": fps,
                "AwbEnable": True,  # Auto white balance
                "AwbMode": 0,  # Auto white balance mode
                "Saturation": 1.0,  # Normal saturation (1.0 = default)
                "Contrast": 1.0,     # Normal contrast (1.0 = default)
                "Brightness": 0.0,   # Default brightness
                "ColourGains": (1.0, 1.0)  # Neutral color gains
            }
        )
        camera.configure(config)
        camera.start()
        time.sleep(2)  # Wait for camera to stabilize
        logger.info("Camera ready")
        
        # Check if FFmpeg is available
        try:
            subprocess.run(['which', 'ffmpeg'], capture_output=True, check=True)
            ffmpeg_available = True
        except:
            ffmpeg_available = False
            logger.warning("FFmpeg not found. Install it for proper video: sudo apt-get install ffmpeg")
        
        if ffmpeg_available:
            # Use FFmpeg for proper MP4 with correct timestamps
            # Use rgb24 instead of bgr24 to avoid color channel swap
            ffmpeg_cmd = [
                'ffmpeg',
                '-y',  # Overwrite output
                '-f', 'rawvideo',
                '-vcodec', 'rawvideo',
                '-s', f'{width}x{height}',
                '-pix_fmt', 'rgb24',  # RGB24 format (not BGR) to preserve natural colors
                '-r', str(fps),  # Input frame rate
                '-i', '-',  # Read from stdin
                '-an',  # No audio
                '-vcodec', 'libx264',
                '-pix_fmt', 'yuv420p',
                '-colorspace', 'bt709',  # Use BT.709 color space (standard for video)
                '-color_primaries', 'bt709',
                '-color_trc', 'bt709',
                '-preset', 'ultrafast',
                '-crf', '23',
                '-r', str(fps),  # Output frame rate
                output_path
            ]
            
            logger.info("Using FFmpeg for recording")
            ffmpeg_process = subprocess.Popen(
                ffmpeg_cmd,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            
            if ffmpeg_process.stdin is None:
                raise Exception("Failed to start FFmpeg")
        else:
            # Fallback to OpenCV VideoWriter
            logger.warning("Using OpenCV VideoWriter (may have speed issues)")
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            video_writer = cv2.VideoWriter(output_path, fourcc, float(fps), (width, height))
            
            if not video_writer.isOpened():
                raise Exception("Failed to open video writer")
        
        # Recording loop
        frame_count = 0
        start_time = time.time()
        frame_interval = 1.0 / fps
        
        logger.info("Recording started...")
        
        while True:
            # Control frame rate to maintain real-world speed
            current_time = time.time()
            target_time = start_time + (frame_count * frame_interval)
            sleep_time = target_time - current_time
            if sleep_time > 0:
                time.sleep(sleep_time)
            
            # Capture frame (RGB888 format from camera)
            frame_rgb = camera.capture_array()
            
            # Write frame directly as RGB (no conversion needed)
            # Picamera2 RGB888 is already in RGB order, so we use it directly
            if ffmpeg_process and ffmpeg_process.stdin:
                try:
                    # Write RGB frame directly (no BGR conversion)
                    ffmpeg_process.stdin.write(frame_rgb.tobytes())
                except BrokenPipeError:
                    logger.error("FFmpeg process ended unexpectedly")
                    break
            elif 'video_writer' in locals():
                # OpenCV VideoWriter expects BGR, so convert only for fallback
                frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
                video_writer.write(frame_bgr)
            
            frame_count += 1
            
            # Log progress every 30 frames
            if frame_count % 30 == 0:
                elapsed = time.time() - start_time
                actual_fps = frame_count / elapsed
                logger.info(f"Recorded {frame_count} frames ({actual_fps:.1f} fps)")
    
    except KeyboardInterrupt:
        logger.info("Stopping recording...")
    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
    finally:
        # Cleanup
        logger.info("Cleaning up...")
        
        # Close FFmpeg
        if ffmpeg_process and ffmpeg_process.stdin:
            logger.info("Finalizing video...")
            try:
                ffmpeg_process.stdin.close()
                ffmpeg_process.wait(timeout=30)
                if ffmpeg_process.returncode == 0:
                    logger.info("Video saved successfully")
                else:
                    stderr = ffmpeg_process.stderr.read().decode('utf-8', errors='ignore')
                    logger.warning(f"FFmpeg exit code: {ffmpeg_process.returncode}")
                    logger.debug(f"FFmpeg output: {stderr}")
            except subprocess.TimeoutExpired:
                logger.warning("FFmpeg encoding timed out")
                ffmpeg_process.kill()
            except Exception as e:
                logger.error(f"Error closing FFmpeg: {e}")
        
        # Close OpenCV VideoWriter
        if 'video_writer' in locals():
            video_writer.release()
        
        # Stop camera
        if camera:
            camera.stop()
            camera.close()
        
        logger.info(f"Recording saved to: {output_path}")


if __name__ == "__main__":
    main()

