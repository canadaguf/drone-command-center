"""
Tracking controller for visual object following.
Calculates movement commands based on detected object position and distance.
"""

import logging
import math
import numpy as np
from typing import Dict, Any, Optional, List, Tuple
from dataclasses import dataclass

logger = logging.getLogger(__name__)


@dataclass
class ControlCommand:
    """Control command for drone movement."""
    vx: float = 0.0  # Forward velocity (m/s)
    vy: float = 0.0  # Right velocity (m/s)
    vz: float = 0.0  # Up velocity (m/s)
    yaw_rate: float = 0.0  # Yaw rate (rad/s)
    
    # Position commands (for corrections)
    target_x: Optional[float] = None  # Target position relative to current (m)
    target_y: Optional[float] = None
    target_z: Optional[float] = None


class TrackingController:
    """Controller for visual object following."""
    
    def __init__(self, config: Dict[str, Any]):
        """Initialize tracking controller.
        
        Args:
            config: Tracking configuration dictionary
        """
        # Camera parameters
        self.camera_width = config.get('camera_width', 640)
        self.camera_height = config.get('camera_height', 480)
        self.fov_horizontal = config.get('fov_horizontal', 120)  # degrees
        self.fov_vertical = config.get('fov_vertical', 90)  # degrees
        
        # Tracking parameters
        self.target_distance = config.get('target_distance', 3.0)  # meters
        self.target_altitude = config.get('target_altitude', 1.5)  # meters
        
        # PID gains
        pid_config = config.get('pid_gains', {})
        self.kp_yaw = pid_config.get('yaw', {}).get('kp', 0.5)
        self.kp_pitch = pid_config.get('pitch', {}).get('kp', 0.3)
        self.kp_roll = pid_config.get('roll', {}).get('kp', 0.3)
        self.kp_distance = pid_config.get('distance', {}).get('kp', 0.2)
        self.kp_altitude = pid_config.get('altitude', {}).get('kp', 15.0)
        
        # Safety thresholds
        safety_config = config.get('safety', {})
        self.yaw_only_threshold = safety_config.get('yaw_only_threshold', 2.0)
        self.obstacle_threshold_forward = safety_config.get('obstacle_threshold_forward', 1.5)
        self.emergency_stop_threshold = safety_config.get('emergency_stop_threshold', 0.8)
        
        # Object reference size (for distance estimation)
        # Average chair dimensions: ~0.5m width, ~1.0m height
        self.reference_width = config.get('reference_width', 0.5)  # meters
        self.reference_height = config.get('reference_height', 1.0)  # meters
        
        # Control state
        self.current_target_id = None
        self.last_detection_time = 0
        self.lost_target_timeout = config.get('lost_target_timeout', 2.0)
        
        # Smoothing
        self.velocity_smoothing = config.get('velocity_smoothing', 0.7)  # 0-1, higher = more smoothing
        self.last_velocity = {'vx': 0.0, 'vy': 0.0, 'vz': 0.0, 'yaw_rate': 0.0}
        
        logger.info(f"TrackingController initialized: target_distance={self.target_distance}m, "
                   f"target_altitude={self.target_altitude}m")
    
    def set_target(self, target_id: int) -> bool:
        """Set target object ID to follow.
        
        Args:
            target_id: ID of object to follow
            
        Returns:
            True if target set successfully
        """
        self.current_target_id = target_id
        logger.info(f"Target set to object ID: {target_id}")
        return True
    
    def stop_tracking(self) -> None:
        """Stop tracking current target."""
        logger.info("Stopping tracking")
        self.current_target_id = None
        self.last_velocity = {'vx': 0.0, 'vy': 0.0, 'vz': 0.0, 'yaw_rate': 0.0}
    
    def is_tracking(self) -> bool:
        """Check if currently tracking a target.
        
        Returns:
            True if tracking, False otherwise
        """
        return self.current_target_id is not None
    
    def update(self, tracked_detections: List[Dict[str, Any]], 
               current_altitude: Optional[float] = None,
               timestamp: float = 0.0) -> Optional[ControlCommand]:
        """Update controller with new detections.
        
        Args:
            tracked_detections: List of tracked detections with 'id' and 'bbox'
            current_altitude: Current altitude AGL (meters)
            timestamp: Current timestamp
            
        Returns:
            ControlCommand if tracking active, None otherwise
        """
        if not self.is_tracking():
            return None
        
        # Find target detection
        target_detection = None
        for det in tracked_detections:
            if det.get('id') == self.current_target_id:
                target_detection = det
                break
        
        if target_detection is None:
            # Target lost
            if timestamp - self.last_detection_time > self.lost_target_timeout:
                logger.warning(f"Target {self.current_target_id} lost for {self.lost_target_timeout}s")
                self.stop_tracking()
                return None
            # Use last known position (dead reckoning)
            return self._create_stop_command()
        
        # Update last detection time
        self.last_detection_time = timestamp
        
        # Calculate control command
        command = self._calculate_command(target_detection, current_altitude)
        
        # Apply smoothing
        if self.velocity_smoothing > 0:
            command.vx = (self.velocity_smoothing * self.last_velocity['vx'] + 
                         (1 - self.velocity_smoothing) * command.vx)
            command.vy = (self.velocity_smoothing * self.last_velocity['vy'] + 
                         (1 - self.velocity_smoothing) * command.vy)
            command.vz = (self.velocity_smoothing * self.last_velocity['vz'] + 
                         (1 - self.velocity_smoothing) * command.vz)
            command.yaw_rate = (self.velocity_smoothing * self.last_velocity['yaw_rate'] + 
                               (1 - self.velocity_smoothing) * command.yaw_rate)
        
        self.last_velocity = {
            'vx': command.vx,
            'vy': command.vy,
            'vz': command.vz,
            'yaw_rate': command.yaw_rate
        }
        
        return command
    
    def _calculate_command(self, detection: Dict[str, Any], 
                          current_altitude: Optional[float]) -> ControlCommand:
        """Calculate control command from detection.
        
        Args:
            detection: Detection dictionary with 'bbox' and 'center'
            current_altitude: Current altitude AGL
            
        Returns:
            ControlCommand
        """
        bbox = detection['bbox']  # (x1, y1, x2, y2)
        center = detection.get('center', 
                              ((bbox[0] + bbox[2]) // 2, (bbox[1] + bbox[3]) // 2))
        
        # Calculate position errors
        frame_center_x = self.camera_width / 2
        frame_center_y = self.camera_height / 2
        
        error_x = center[0] - frame_center_x  # Positive = object to the right
        error_y = center[1] - frame_center_y  # Positive = object below
        
        # Calculate distance from bbox size
        bbox_width = bbox[2] - bbox[0]
        bbox_height = bbox[3] - bbox[1]
        estimated_distance = self._estimate_distance(bbox_width, bbox_height)
        
        # Safety check
        if estimated_distance < self.emergency_stop_threshold:
            logger.warning(f"Emergency stop: object too close ({estimated_distance:.2f}m)")
            return self._create_stop_command()
        
        # Calculate control outputs
        command = ControlCommand()
        
        # Yaw control (horizontal centering)
        # Convert pixel error to angle error
        angle_error_x = (error_x / self.camera_width) * math.radians(self.fov_horizontal)
        command.yaw_rate = -self.kp_yaw * angle_error_x  # Negative for correct direction
        
        # Forward/backward control (distance maintenance)
        distance_error = estimated_distance - self.target_distance
        if estimated_distance < self.yaw_only_threshold:
            # Too close - only yaw, no forward movement
            command.vx = 0.0
        else:
            # Use proportional control for forward/backward
            command.vx = -self.kp_distance * distance_error  # Negative = forward when too far
        
        # Limit forward velocity based on safety
        if estimated_distance < self.obstacle_threshold_forward:
            command.vx = min(command.vx, 0.0)  # Only allow backward movement
        
        # Altitude control (vertical centering + altitude maintenance)
        if current_altitude is not None:
            altitude_error = self.target_altitude - current_altitude
            # Vertical centering (pitch)
            angle_error_y = (error_y / self.camera_height) * math.radians(self.fov_vertical)
            pitch_velocity = -self.kp_pitch * angle_error_y
            
            # Altitude maintenance
            altitude_velocity = self.kp_altitude * altitude_error * 0.1  # Scale down
            
            # Combine (prioritize altitude maintenance)
            command.vz = altitude_velocity + pitch_velocity * 0.3
        else:
            # No altitude data - use only vertical centering
            angle_error_y = (error_y / self.camera_height) * math.radians(self.fov_vertical)
            command.vz = -self.kp_pitch * angle_error_y * 0.5  # Reduced gain
        
        # Roll control (lateral movement) - minimal for centering
        # Usually not needed if yaw is working well
        command.vy = 0.0
        
        # Limit velocities
        command.vx = max(-1.0, min(1.0, command.vx))  # m/s
        command.vy = max(-0.5, min(0.5, command.vy))  # m/s
        command.vz = max(-0.5, min(0.5, command.vz))  # m/s
        command.yaw_rate = max(-0.5, min(0.5, command.yaw_rate))  # rad/s
        
        return command
    
    def _estimate_distance(self, bbox_width: int, bbox_height: int) -> float:
        """Estimate distance to object from bounding box size.
        
        Uses fixed reference size and camera parameters.
        
        Args:
            bbox_width: Bounding box width in pixels
            bbox_height: Bounding box height in pixels
            
        Returns:
            Estimated distance in meters
        """
        # Use height for distance estimation (more reliable)
        # Formula: distance = (reference_height * focal_length) / (bbox_height_pixels * pixel_size)
        # Simplified: distance = (reference_height * image_height) / (bbox_height * tan(FOV/2))
        
        # Calculate focal length equivalent
        fov_rad = math.radians(self.fov_vertical)
        focal_length_pixels = self.camera_height / (2 * math.tan(fov_rad / 2))
        
        # Estimate distance from height
        if bbox_height > 0:
            distance_height = (self.reference_height * focal_length_pixels) / bbox_height
        else:
            distance_height = self.target_distance
        
        # Estimate distance from width (as backup)
        if bbox_width > 0:
            fov_h_rad = math.radians(self.fov_horizontal)
            focal_length_pixels_h = self.camera_width / (2 * math.tan(fov_h_rad / 2))
            distance_width = (self.reference_width * focal_length_pixels_h) / bbox_width
        else:
            distance_width = self.target_distance
        
        # Use average of both estimates
        estimated_distance = (distance_height + distance_width) / 2
        
        # Clamp to reasonable range
        estimated_distance = max(0.5, min(20.0, estimated_distance))
        
        return estimated_distance
    
    def _create_stop_command(self) -> ControlCommand:
        """Create stop command (zero velocities)."""
        return ControlCommand(vx=0.0, vy=0.0, vz=0.0, yaw_rate=0.0)
    
    def get_status(self) -> Dict[str, Any]:
        """Get controller status.
        
        Returns:
            Dictionary with status information
        """
        return {
            'tracking': self.is_tracking(),
            'target_id': self.current_target_id,
            'target_distance': self.target_distance,
            'target_altitude': self.target_altitude
        }

