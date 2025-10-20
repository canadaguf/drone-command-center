"""
Tracking controller implementing SPF-inspired algorithm.
Converts 2D person detection to 3D drone movement commands.
"""

import math
import logging
from typing import Dict, Any, Optional, Tuple, List
import numpy as np

from .pid_controller import PIDManager
from ..sensors.telemetry import TelemetryCollector

logger = logging.getLogger(__name__)

class TrackingController:
    """Tracking controller for person following."""
    
    def __init__(self, config: Dict[str, Any], pid_manager: PIDManager, 
                 telemetry_collector: TelemetryCollector):
        """Initialize tracking controller.
        
        Args:
            config: Tracking configuration
            pid_manager: PID controller manager
            telemetry_collector: Telemetry data collector
        """
        self.config = config
        self.pid_manager = pid_manager
        self.telemetry = telemetry_collector
        
        # Distance modes
        self.distance_modes = config.get('distance_modes', {
            'close': 2.0,
            'medium': 4.0,
            'far': 6.0
        })
        self.current_distance_mode = config.get('default_mode', 'medium')
        self.target_distance = self.distance_modes[self.current_distance_mode]
        
        # Altitude control
        self.altitude_config = config.get('altitude', {})
        self.auto_adjust_altitude = self.altitude_config.get('auto_adjust', True)
        self.min_height = self.altitude_config.get('min_height', 1.0)
        self.max_height = self.altitude_config.get('max_height', 10.0)
        
        # Camera parameters
        self.camera_width = 640  # Will be updated from camera
        self.camera_height = 480
        self.fov_horizontal = 120  # degrees
        self.fov_vertical = 90  # degrees
        
        # Tracking state
        self.current_target = None
        self.tracking_active = False
        self.last_target_time = 0
        self.target_lost_time = 0
        
        # Lost target handling
        self.lost_target_config = config.get('lost_target', {})
        self.hover_timeout = self.lost_target_config.get('hover_timeout', 2.0)
        self.land_timeout = self.lost_target_config.get('land_timeout', 10.0)
        
        logger.info(f"Tracking controller initialized: mode={self.current_distance_mode}, target_distance={self.target_distance}m")
    
    def set_target(self, person_id: int) -> bool:
        """Set target person for tracking.
        
        Args:
            person_id: ID of person to track
            
        Returns:
            True if target set successfully, False otherwise
        """
        self.current_target = person_id
        self.tracking_active = True
        self.last_target_time = 0
        self.target_lost_time = 0
        
        logger.info(f"Tracking target set: person {person_id}")
        return True
    
    def stop_tracking(self) -> None:
        """Stop tracking current target."""
        self.current_target = None
        self.tracking_active = False
        self.target_lost_time = 0
        
        logger.info("Tracking stopped")
    
    def set_distance_mode(self, mode: str) -> bool:
        """Set distance tracking mode.
        
        Args:
            mode: Distance mode ('close', 'medium', 'far')
            
        Returns:
            True if mode set successfully, False otherwise
        """
        if mode not in self.distance_modes:
            logger.error(f"Invalid distance mode: {mode}")
            return False
        
        self.current_distance_mode = mode
        self.target_distance = self.distance_modes[mode]
        
        logger.info(f"Distance mode set to {mode}: {self.target_distance}m")
        return True
    
    def update(self, detections: List[Dict[str, Any]], 
                tof_forward: Optional[float] = None,
                tof_down: Optional[float] = None) -> Dict[str, Any]:
        """Update tracking controller with new detections.
        
        Args:
            detections: List of person detections
            tof_forward: Forward TOF distance
            tof_down: Downward TOF distance
            
        Returns:
            Dictionary containing tracking commands and status
        """
        if not self.tracking_active or not self.current_target:
            return self._get_idle_commands()
        
        # Find target person
        target_detection = self._find_target_detection(detections)
        
        if target_detection is None:
            return self._handle_lost_target()
        
        # Update target tracking
        self.last_target_time = 0
        self.target_lost_time = 0
        
        # Calculate tracking commands
        commands = self._calculate_tracking_commands(
            target_detection, tof_forward, tof_down
        )
        
        # Update telemetry
        self.telemetry.update_tracking_status("TRACKING")
        
        return commands
    
    def _find_target_detection(self, detections: List[Dict[str, Any]]) -> Optional[Dict[str, Any]]:
        """Find target person in detections.
        
        Args:
            detections: List of person detections
            
        Returns:
            Target detection if found, None otherwise
        """
        for detection in detections:
            if detection.get('id') == self.current_target:
                return detection
        return None
    
    def _handle_lost_target(self) -> Dict[str, Any]:
        """Handle lost target situation.
        
        Returns:
            Dictionary containing appropriate commands
        """
        self.target_lost_time += 0.1  # Assuming 10Hz update rate
        
        if self.target_lost_time <= self.hover_timeout:
            # Hover in place
            self.telemetry.update_tracking_status("LOST")
            return self._get_hover_commands()
        elif self.target_lost_time <= self.land_timeout:
            # Continue hovering
            self.telemetry.update_tracking_status("HOVERING")
            return self._get_hover_commands()
        else:
            # Land
            self.telemetry.update_tracking_status("LANDING")
            return self._get_land_commands()
    
    def _calculate_tracking_commands(self, detection: Dict[str, Any], 
                                   tof_forward: Optional[float],
                                   tof_down: Optional[float]) -> Dict[str, Any]:
        """Calculate tracking commands from detection.
        
        Args:
            detection: Target person detection
            tof_forward: Forward TOF distance
            tof_down: Downward TOF distance
            
        Returns:
            Dictionary containing tracking commands
        """
        # Get detection center
        center_x, center_y = detection['center']
        
        # Calculate 2D errors
        image_center_x = self.camera_width / 2
        image_center_y = self.camera_height / 2
        
        error_x = center_x - image_center_x  # Horizontal offset
        error_y = center_y - image_center_y  # Vertical offset
        
        # Convert to angular errors
        fov_h_rad = math.radians(self.fov_horizontal)
        fov_v_rad = math.radians(self.fov_vertical)
        
        angle_yaw = (error_x / self.camera_width) * fov_h_rad
        angle_pitch = (error_y / self.camera_height) * fov_v_rad
        
        # Determine distance
        distance = self._estimate_distance(detection, tof_forward)
        
        # Calculate 3D displacement
        lateral_x = distance * math.tan(angle_yaw)
        lateral_y = distance * math.tan(angle_pitch)
        
        # Calculate PID outputs
        pid_errors = {
            'yaw': angle_yaw,
            'pitch': lateral_y,
            'roll': lateral_x,
            'distance': distance - self.target_distance
        }
        
        pid_outputs = self.pid_manager.update(pid_errors)
        
        # Apply altitude adjustment
        altitude_adjustment = 0
        if self.auto_adjust_altitude:
            altitude_adjustment = self._calculate_altitude_adjustment(
                error_y, tof_down
            )
        
        # Generate RC commands
        roll_cmd = int(np.clip(pid_outputs['roll'], -100, 100))
        pitch_cmd = int(np.clip(pid_outputs['pitch'], -100, 100))
        yaw_cmd = int(np.clip(pid_outputs['yaw'], -100, 100))
        throttle_cmd = int(np.clip(50 + altitude_adjustment, 0, 100))
        
        return {
            'roll': roll_cmd,
            'pitch': pitch_cmd,
            'yaw': yaw_cmd,
            'throttle': throttle_cmd,
            'distance': distance,
            'target_distance': self.target_distance,
            'tracking_active': True
        }
    
    def _estimate_distance(self, detection: Dict[str, Any], 
                          tof_forward: Optional[float]) -> float:
        """Estimate distance to target.
        
        Args:
            detection: Target detection
            tof_forward: Forward TOF distance
            
        Returns:
            Estimated distance in meters
        """
        # Use TOF if available and within range
        if tof_forward is not None and 0.5 <= tof_forward <= 4.0:
            return tof_forward
        
        # Estimate from bbox size (simplified)
        bbox = detection['bbox']
        bbox_height = bbox[3] - bbox[1]  # y2 - y1
        
        # Simple heuristic: larger bbox = closer person
        height_ratio = bbox_height / self.camera_height
        
        # Rough distance estimation (calibrate empirically)
        if height_ratio > 0.5:
            return 1.0
        elif height_ratio > 0.3:
            return 2.0
        elif height_ratio > 0.2:
            return 3.0
        elif height_ratio > 0.1:
            return 5.0
        else:
            return 8.0
    
    def _calculate_altitude_adjustment(self, vertical_error: float, 
                                     tof_down: Optional[float]) -> float:
        """Calculate altitude adjustment.
        
        Args:
            vertical_error: Vertical pixel error
            tof_down: Downward TOF distance
            
        Returns:
            Altitude adjustment (-50 to 50)
        """
        # Simple altitude adjustment based on vertical error
        adjustment = -vertical_error * 0.1  # Scale factor
        
        # Apply bounds
        adjustment = max(-50, min(50, adjustment))
        
        # Check for obstacles below
        if tof_down is not None and tof_down < 2.0:
            # Obstacle detected below, climb
            adjustment = max(adjustment, 20)
        
        return adjustment
    
    def _get_idle_commands(self) -> Dict[str, Any]:
        """Get idle commands (no tracking).
        
        Returns:
            Dictionary containing idle commands
        """
        return {
            'roll': 0,
            'pitch': 0,
            'yaw': 0,
            'throttle': 0,
            'distance': 0,
            'target_distance': self.target_distance,
            'tracking_active': False
        }
    
    def _get_hover_commands(self) -> Dict[str, Any]:
        """Get hover commands (position hold).
        
        Returns:
            Dictionary containing hover commands
        """
        return {
            'roll': 0,
            'pitch': 0,
            'yaw': 0,
            'throttle': 0,  # Maintain current altitude
            'distance': 0,
            'target_distance': self.target_distance,
            'tracking_active': False,
            'hover': True
        }
    
    def _get_land_commands(self) -> Dict[str, Any]:
        """Get land commands.
        
        Returns:
            Dictionary containing land commands
        """
        return {
            'roll': 0,
            'pitch': 0,
            'yaw': 0,
            'throttle': -50,  # Descend
            'distance': 0,
            'target_distance': self.target_distance,
            'tracking_active': False,
            'land': True
        }
    
    def get_status(self) -> Dict[str, Any]:
        """Get tracking controller status.
        
        Returns:
            Dictionary containing status information
        """
        return {
            'tracking_active': self.tracking_active,
            'current_target': self.current_target,
            'distance_mode': self.current_distance_mode,
            'target_distance': self.target_distance,
            'target_lost_time': self.target_lost_time,
            'hover_timeout': self.hover_timeout,
            'land_timeout': self.land_timeout,
            'auto_adjust_altitude': self.auto_adjust_altitude
        }
    
    def reset_pid(self) -> None:
        """Reset PID controllers."""
        self.pid_manager.reset_all()
        logger.info("PID controllers reset")
    
    def set_camera_params(self, width: int, height: int, 
                         fov_h: float, fov_v: float) -> None:
        """Set camera parameters.
        
        Args:
            width: Camera width
            height: Camera height
            fov_h: Horizontal FOV in degrees
            fov_v: Vertical FOV in degrees
        """
        self.camera_width = width
        self.camera_height = height
        self.fov_horizontal = fov_h
        self.fov_vertical = fov_v
        
        logger.info(f"Camera parameters updated: {width}x{height}, FOV={fov_h}°x{fov_v}°")
