"""
Camera manager using Picamera2.
Handles camera initialization, frame capture, and configuration.
"""

import time
import logging
from typing import Dict, Any, Optional, Tuple
from picamera2 import Picamera2
import numpy as np

logger = logging.getLogger(__name__)

class CameraManager:
    """Camera manager for RPi Camera Module 3."""
    
    def __init__(self, config: Dict[str, Any]):
        """Initialize camera manager.
        
        Args:
            config: Camera configuration dictionary
        """
        self.config = config
        self.width = config.get('width', 640)
        self.height = config.get('height', 480)
        self.fps = config.get('fps', 30)
        self.fov_horizontal = config.get('fov_horizontal', 120)
        self.fov_vertical = config.get('fov_vertical', 90)
        
        self.picam2 = None
        self.initialized = False
        self.last_frame = None
        self.last_frame_time = 0
        
    def initialize(self) -> bool:
        """Initialize camera.
        
        Returns:
            True if initialization successful, False otherwise
        """
        try:
            logger.info(f"Initializing camera: {self.width}x{self.height} @ {self.fps}fps")
            
            self.picam2 = Picamera2()
            
            # Create camera configuration
            config = self.picam2.create_preview_configuration(
                main={
                    "size": (self.width, self.height),
                    "format": "RGB888"
                },
                controls={
                    "FrameRate": self.fps
                }
            )
            
            # Configure camera
            self.picam2.configure(config)
            
            # Start camera
            self.picam2.start()
            
            # Wait for camera to stabilize
            time.sleep(2)
            
            self.initialized = True
            logger.info("Camera initialized successfully")
            return True
            
        except Exception as e:
            logger.error(f"Failed to initialize camera: {e}")
            return False
    
    def capture_frame(self) -> Optional[np.ndarray]:
        """Capture a frame from camera.
        
        Returns:
            Frame as numpy array (RGB), None if capture failed
        """
        if not self.initialized or not self.picam2:
            logger.error("Camera not initialized")
            return None
        
        try:
            frame = self.picam2.capture_array()
            self.last_frame = frame
            self.last_frame_time = time.time()
            return frame
            
        except Exception as e:
            logger.error(f"Failed to capture frame: {e}")
            return None
    
    def get_frame(self) -> Optional[np.ndarray]:
        """Get last captured frame.
        
        Returns:
            Last frame as numpy array, None if not available
        """
        return self.last_frame
    
    def get_frame_info(self) -> Dict[str, Any]:
        """Get frame information.
        
        Returns:
            Dictionary containing frame info
        """
        return {
            'width': self.width,
            'height': self.height,
            'fps': self.fps,
            'fov_horizontal': self.fov_horizontal,
            'fov_vertical': self.fov_vertical,
            'initialized': self.initialized,
            'last_frame_time': self.last_frame_time,
            'has_frame': self.last_frame is not None
        }
    
    def set_fps(self, fps: int) -> bool:
        """Set camera FPS.
        
        Args:
            fps: Target FPS
            
        Returns:
            True if successful, False otherwise
        """
        if not self.initialized:
            logger.error("Camera not initialized")
            return False
        
        try:
            self.picam2.set_controls({"FrameRate": fps})
            self.fps = fps
            logger.info(f"Camera FPS set to {fps}")
            return True
        except Exception as e:
            logger.error(f"Failed to set FPS: {e}")
            return False
    
    def set_resolution(self, width: int, height: int) -> bool:
        """Set camera resolution.
        
        Args:
            width: Frame width
            height: Frame height
            
        Returns:
            True if successful, False otherwise
        """
        if not self.initialized:
            logger.error("Camera not initialized")
            return False
        
        try:
            # Stop camera
            self.picam2.stop()
            
            # Create new configuration
            config = self.picam2.create_preview_configuration(
                main={
                    "size": (width, height),
                    "format": "RGB888"
                },
                controls={
                    "FrameRate": self.fps
                }
            )
            
            # Configure and restart
            self.picam2.configure(config)
            self.picam2.start()
            
            self.width = width
            self.height = height
            
            logger.info(f"Camera resolution set to {width}x{height}")
            return True
            
        except Exception as e:
            logger.error(f"Failed to set resolution: {e}")
            return False
    
    def get_camera_info(self) -> Dict[str, Any]:
        """Get camera information.
        
        Returns:
            Dictionary containing camera info
        """
        info = {
            'width': self.width,
            'height': self.height,
            'fps': self.fps,
            'fov_horizontal': self.fov_horizontal,
            'fov_vertical': self.fov_vertical,
            'initialized': self.initialized
        }
        
        if self.picam2:
            try:
                # Get camera properties
                info.update({
                    'camera_model': self.picam2.camera_properties.get('Model', 'Unknown'),
                    'sensor_mode': self.picam2.camera_properties.get('SensorMode', 'Unknown')
                })
            except:
                pass
        
        return info
    
    def cleanup(self) -> None:
        """Cleanup camera resources."""
        if self.picam2:
            try:
                self.picam2.stop()
                self.picam2.close()
                logger.info("Camera cleaned up")
            except Exception as e:
                logger.error(f"Error cleaning up camera: {e}")
        
        self.initialized = False
