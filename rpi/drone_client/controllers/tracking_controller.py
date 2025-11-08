"""
Tracking controller implementing SPF-inspired algorithm.
Converts 2D person detection to 3D drone movement commands.
"""

import math
import logging
from typing import Dict, Any, Optional, Tuple, List
from collections import deque
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
        self.target_altitude = self.altitude_config.get('target_altitude', 1.5)  # Target altitude to maintain
        self.min_height = self.altitude_config.get('min_height', 1.0)
        self.max_height = self.altitude_config.get('max_height', 10.0)
        
        # Base hover throttle (will be dynamically adjusted, but this is starting point)
        # This should be calibrated for your specific drone
        self.base_hover_throttle = self.altitude_config.get('base_hover_throttle', 50)
        
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
        
        # Safety: Yaw-only mode for close targets (SPF-inspired)
        # Safety config is nested under tracking.safety
        self.safety_config = config.get('safety', {})
        self.yaw_only_threshold = self.safety_config.get('yaw_only_threshold', 2.0)  # meters
        
        # Obstacle avoidance thresholds
        self.obstacle_threshold_forward = self.safety_config.get('obstacle_threshold_forward', 1.5)  # meters
        self.obstacle_threshold_down = self.safety_config.get('obstacle_threshold_down', 0.5)  # meters
        self.emergency_stop_threshold = self.safety_config.get('emergency_stop_threshold', 0.8)  # meters
        
        # Action history for oscillation prevention
        self.action_history = deque(maxlen=5)
        
        logger.info(f"Tracking controller initialized: mode={self.current_distance_mode}, target_distance={self.target_distance}m")
        logger.info(f"Safety: Yaw-only mode threshold = {self.yaw_only_threshold}m")
    
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
        
        # Update telemetry to show hover status
        self.telemetry.update_tracking_status("HOVERING")
        
        logger.info("Tracking stopped - entering hover mode")
    
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
                tof_down: Optional[float] = None,
                current_altitude: Optional[float] = None) -> Dict[str, Any]:
        """Update tracking controller with new detections.
        
        Args:
            detections: List of person detections
            tof_forward: Forward TOF distance
            tof_down: Downward TOF distance
            current_altitude: Current altitude from MAVLink (relative_alt) in meters
            
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
            target_detection, tof_forward, tof_down, current_altitude
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
                                   tof_down: Optional[float],
                                   current_altitude: Optional[float] = None) -> Dict[str, Any]:
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
        
        # Determine distance (prioritize TOF if available and valid)
        distance = self._estimate_distance(detection, tof_forward)
        
        # OBSTACLE AVOIDANCE: Check forward TOF sensor for obstacles
        obstacle_detected_forward = False
        if tof_forward is not None:
            if tof_forward < self.emergency_stop_threshold:
                # Emergency stop - obstacle very close
                logger.warning(f"EMERGENCY STOP: Forward obstacle detected at {tof_forward:.2f}m")
                return self._get_emergency_stop_commands(distance, tof_down, current_altitude)
            elif tof_forward < self.obstacle_threshold_forward:
                # Obstacle detected - prevent forward movement
                obstacle_detected_forward = True
                logger.warning(f"Obstacle detected forward at {tof_forward:.2f}m - preventing forward movement")
        
        # SAFETY: Yaw-only mode for close targets (SPF-inspired)
        # Prevents forward movement when target is too close to avoid collisions
        # OR when obstacle is detected by forward TOF sensor
        if distance < self.yaw_only_threshold or obstacle_detected_forward:
            logger.warning(f"Target too close ({distance:.2f}m < {self.yaw_only_threshold}m) - YAW ONLY mode")
            
            # Only calculate yaw correction
            yaw_error = angle_yaw
            pid_outputs = self.pid_manager.update({'yaw': yaw_error})
            yaw_cmd = int(np.clip(pid_outputs['yaw'], -100, 100))
            
            # Calculate throttle dynamically based on altitude
            throttle_cmd = self._calculate_throttle(
                current_altitude, tof_down, error_y
            )
            
            # Store action history
            action = {
                'roll': 0,
                'pitch': 0,  # No forward movement
                'yaw': yaw_cmd,
                'throttle': throttle_cmd,
                'distance': distance,
                'yaw_only': True
            }
            self.action_history.append(action)
            
            return {
                'roll': 0,  # No lateral movement
                'pitch': 0,  # No forward movement - SAFETY
                'yaw': yaw_cmd,
                'throttle': throttle_cmd,
                'distance': distance,
                'target_distance': self.target_distance,
                'tracking_active': True,
                'yaw_only': True  # Flag for telemetry/logging
            }
        
        # Normal operation: full 3D tracking
        # Calculate 3D displacement using improved projection
        lateral_x, lateral_y = self._reverse_project_point(
            (center_x, center_y), distance
        )
        
        # Calculate PID outputs
        pid_errors = {
            'yaw': angle_yaw,
            'pitch': lateral_y,
            'roll': lateral_x,
            'distance': distance - self.target_distance
        }
        
        pid_outputs = self.pid_manager.update(pid_errors)
        
        # Generate RC commands
        roll_cmd = int(np.clip(pid_outputs['roll'], -100, 100))
        pitch_cmd = int(np.clip(pid_outputs['pitch'], -100, 100))
        yaw_cmd = int(np.clip(pid_outputs['yaw'], -100, 100))
        
        # Calculate throttle dynamically based on altitude
        throttle_cmd = self._calculate_throttle(
            current_altitude, tof_down, error_y
        )
        
        # OBSTACLE AVOIDANCE: Prevent forward movement if obstacle detected
        if obstacle_detected_forward and pitch_cmd > 0:
            logger.debug(f"Blocking forward movement (pitch={pitch_cmd}) due to obstacle at {tof_forward:.2f}m")
            pitch_cmd = 0  # Block forward movement
        
        # Oscillation prevention: check action history
        if len(self.action_history) >= 2:
            last_action = self.action_history[-1]
            # If yaw direction changed rapidly, reduce gain
            if abs(yaw_cmd) > 0 and abs(last_action.get('yaw', 0)) > 0:
                if (yaw_cmd * last_action['yaw']) < 0:  # Opposite signs
                    logger.debug("Oscillation detected in yaw - reducing gain")
                    yaw_cmd = int(yaw_cmd * 0.5)  # Reduce by 50%
            
            # Check for pitch oscillation
            if abs(pitch_cmd) > 0 and abs(last_action.get('pitch', 0)) > 0:
                if (pitch_cmd * last_action['pitch']) < 0:  # Opposite signs
                    logger.debug("Oscillation detected in pitch - reducing gain")
                    pitch_cmd = int(pitch_cmd * 0.5)
        
        # Store action history
        action = {
            'roll': roll_cmd,
            'pitch': pitch_cmd,
            'yaw': yaw_cmd,
            'throttle': throttle_cmd,
            'distance': distance,
            'yaw_only': False
        }
        self.action_history.append(action)
        
        return {
            'roll': roll_cmd,
            'pitch': pitch_cmd,
            'yaw': yaw_cmd,
            'throttle': throttle_cmd,
            'distance': distance,
            'target_distance': self.target_distance,
            'tracking_active': True,
            'yaw_only': False
        }
    
    def _estimate_distance(self, detection: Dict[str, Any], 
                          tof_forward: Optional[float]) -> float:
        """Estimate distance to target with improved accuracy (SPF-inspired).
        
        Prioritizes TOF sensor reading when available and valid.
        
        Args:
            detection: Target detection
            tof_forward: Forward TOF distance in meters
            
        Returns:
            Estimated distance in meters
        """
        # Use TOF if available and within valid range (most accurate)
        # TOF sensors are most reliable between 0.3m and 4.0m
        if tof_forward is not None and 0.3 <= tof_forward <= 4.0:
            logger.debug(f"Using TOF distance: {tof_forward:.2f}m")
            return tof_forward
        
        # Estimate from bbox size with improved calibration
        bbox = detection['bbox']
        bbox_height = bbox[3] - bbox[1]  # y2 - y1
        
        # Height ratio: larger bbox = closer person
        height_ratio = bbox_height / self.camera_height
        
        # Improved distance estimation with smoother transitions
        # Based on SPF project's calibration approach
        if height_ratio > 0.5:
            return 1.0  # Very close
        elif height_ratio > 0.3:
            return 2.0  # Close
        elif height_ratio > 0.2:
            return 3.0  # Medium-close
        elif height_ratio > 0.15:
            # Smooth interpolation for medium distances
            # Linear interpolation between 3.0 and 5.0
            t = (height_ratio - 0.15) / (0.2 - 0.15)
            return 3.0 + t * 2.0
        elif height_ratio > 0.1:
            return 5.0  # Medium-far
        elif height_ratio > 0.05:
            # Smooth interpolation for far distances
            t = (height_ratio - 0.05) / (0.1 - 0.05)
            return 5.0 + t * 3.0
        else:
            return 8.0  # Very far
    
    def _reverse_project_point(self, point_2d: Tuple[float, float], 
                               depth: float) -> Tuple[float, float]:
        """Reverse project 2D point to 3D space (SPF-inspired improved projection).
        
        Uses reference point at 35% from top for better drone perspective.
        
        Args:
            point_2d: 2D image point (x, y)
            depth: Estimated depth in meters
            
        Returns:
            Tuple of (lateral_x, lateral_y) in 3D space
        """
        center_x = self.camera_width / 2
        reference_y = self.camera_height * 0.35  # 35% from top (SPF reference)
        
        # Normalized coordinates
        x_normalized = (point_2d[0] - center_x) / (self.camera_width / 2)
        y_normalized = (reference_y - point_2d[1]) / (self.camera_height / 2)
        
        # Depth adjustment based on vertical position (closer if lower in image)
        depth_factor = 1.0 + (y_normalized * 0.5)
        adjusted_depth = depth * depth_factor
        
        # Calculate 3D coordinates with FOV
        fov_h_rad = math.radians(self.fov_horizontal / 2)
        fov_v_rad = math.radians(self.fov_vertical / 2)
        
        lateral_x = adjusted_depth * x_normalized * math.tan(fov_h_rad)
        lateral_y = adjusted_depth * y_normalized * math.tan(fov_v_rad)
        
        return (lateral_x, lateral_y)
    
    def _calculate_throttle(self, current_altitude: Optional[float],
                           tof_down: Optional[float],
                           vertical_error: float = 0.0) -> int:
        """Calculate throttle dynamically based on altitude feedback.
        
        Uses PID control to maintain target altitude. This prevents hardcoded
        throttle values that don't account for different drone weights/configurations.
        
        Args:
            current_altitude: Current altitude from MAVLink (relative_alt) in meters
            tof_down: Downward TOF distance in meters (fallback if MAVLink unavailable)
            vertical_error: Vertical pixel error from visual tracking
            
        Returns:
            Throttle command (0-100)
        """
        # Determine actual altitude (prefer MAVLink, fallback to TOF)
        actual_altitude = current_altitude
        if actual_altitude is None and tof_down is not None:
            # Use TOF as fallback (less accurate but better than nothing)
            actual_altitude = tof_down
            logger.debug(f"Using TOF altitude ({tof_down:.2f}m) as fallback")
        
        # If no altitude data available, use base hover throttle
        if actual_altitude is None:
            logger.warning("No altitude data available - using base hover throttle")
            return self.base_hover_throttle
        
        # Calculate altitude error (target - actual)
        altitude_error = self.target_altitude - actual_altitude
        
        # Use PID controller for altitude if available
        if 'altitude' in self.pid_manager.controllers:
            pid_output = self.pid_manager.controllers['altitude'].update(altitude_error)
            # PID output is in range -100 to 100, convert to throttle adjustment
            throttle_adjustment = pid_output
        else:
            # Fallback: simple proportional control
            kp = 15.0  # Proportional gain
            throttle_adjustment = kp * altitude_error
        
        # Add visual tracking adjustment (small contribution)
        visual_adjustment = -vertical_error * 0.05  # Small scale factor
        
        # Calculate base throttle (start from hover)
        throttle = self.base_hover_throttle + throttle_adjustment + visual_adjustment
        
        # Apply safety limits based on TOF downward sensor
        if tof_down is not None:
            if tof_down < self.obstacle_threshold_down:
                # Emergency: too close to ground - force climb
                logger.warning(f"Ground too close ({tof_down:.2f}m) - forcing climb")
                throttle = max(throttle, self.base_hover_throttle + 30)
            elif tof_down < 0.8:
                # Low altitude - ensure minimum climb
                throttle = max(throttle, self.base_hover_throttle + 15)
            elif tof_down > self.max_height:
                # Too high - limit maximum throttle
                throttle = min(throttle, self.base_hover_throttle - 10)
        
        # Clamp to valid range (0-100)
        throttle = int(np.clip(throttle, 0, 100))
        
        logger.debug(f"Throttle: alt={actual_altitude:.2f}m, target={self.target_altitude:.2f}m, "
                    f"error={altitude_error:.2f}m, throttle={throttle}")
        
        return throttle
    
    def _calculate_altitude_adjustment(self, vertical_error: float, 
                                     tof_down: Optional[float]) -> float:
        """Calculate altitude adjustment with enhanced TOF integration.
        
        Uses downward TOF sensor for precise altitude control and obstacle avoidance.
        
        Args:
            vertical_error: Vertical pixel error (positive = target below center)
            tof_down: Downward TOF distance in meters
            
        Returns:
            Altitude adjustment (-50 to 50, positive = climb, negative = descend)
        """
        # Base adjustment from visual tracking (target position in frame)
        adjustment = -vertical_error * 0.1  # Scale factor
        
        # ENHANCED: Use downward TOF sensor for altitude control
        if tof_down is not None:
            # Check for ground/obstacle below
            if tof_down < self.obstacle_threshold_down:
                # Emergency: too close to ground/obstacle - climb immediately
                logger.warning(f"Ground too close ({tof_down:.2f}m) - climbing")
                adjustment = 50  # Maximum climb
            elif tof_down < 0.8:
                # Close to ground - prioritize climbing
                adjustment = max(adjustment, 30)
            elif tof_down < 1.2:
                # Low altitude - gentle climb preference
                adjustment = max(adjustment, 10)
            elif tof_down > 3.0:
                # High altitude - allow descent if needed
                adjustment = min(adjustment, 0)
            
            # Maintain minimum safe altitude
            if tof_down < self.min_height:
                adjustment = max(adjustment, 20)
            
            # Respect maximum altitude
            if tof_down > self.max_height:
                adjustment = min(adjustment, -20)
        
        # Apply bounds
        adjustment = max(-50, min(50, adjustment))
        
        return adjustment
    
    def _get_idle_commands(self) -> Dict[str, Any]:
        """Get idle commands (no tracking) - sends hover commands.
        
        When tracking stops, we send neutral RC commands to maintain hover.
        The flight controller in GUIDED mode will maintain position.
        
        Returns:
            Dictionary containing hover commands
        """
        # Return hover commands when idle (maintain current position)
        return {
            'roll': 0,      # Neutral roll (1500 RC)
            'pitch': 0,     # Neutral pitch (1500 RC)
            'yaw': 0,       # Neutral yaw (1500 RC)
            'throttle': 50, # Mid throttle to maintain altitude (1500 RC)
            'distance': 0,
            'target_distance': self.target_distance,
            'tracking_active': False,
            'hover': True
        }
    
    def _get_hover_commands(self) -> Dict[str, Any]:
        """Get hover commands (position hold).
        
        Returns:
            Dictionary containing hover commands
        """
        # SAFETY: Hover must maintain altitude using dynamic throttle
        # Use base hover throttle as minimum
        return {
            'roll': 0,
            'pitch': 0,
            'yaw': 0,
            'throttle': self.base_hover_throttle,  # Dynamic hover throttle
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
    
    def _get_emergency_stop_commands(self, distance: float, 
                                     tof_down: Optional[float],
                                     current_altitude: Optional[float] = None) -> Dict[str, Any]:
        """Get emergency stop commands when obstacle detected.
        
        CRITICAL: Always maintains hover throttle (50) to prevent falling.
        Only allows climb if needed for ground avoidance, never descent.
        
        Args:
            distance: Current estimated distance to target
            tof_down: Downward TOF distance
            
        Returns:
            Dictionary containing emergency stop commands
        """
        # SAFETY: Emergency stop MUST maintain altitude to prevent crash
        # Use dynamic throttle calculation, but ensure minimum hover throttle
        throttle_cmd = self._calculate_throttle(current_altitude, tof_down, 0.0)
        
        # Emergency stop: ensure we never go below hover throttle
        throttle_cmd = max(throttle_cmd, self.base_hover_throttle)
        
        # If ground too close, force climb
        if tof_down is not None and tof_down < self.obstacle_threshold_down:
            logger.warning(f"Emergency stop: Ground too close ({tof_down:.2f}m) - forcing climb")
            throttle_cmd = max(throttle_cmd, self.base_hover_throttle + 30)
        
        logger.info(f"Emergency stop: Maintaining altitude (throttle={throttle_cmd}, tof_down={tof_down})")
        
        return {
            'roll': 0,  # No lateral movement
            'pitch': 0,  # No forward movement - EMERGENCY STOP
            'yaw': 0,   # No rotation
            'throttle': throttle_cmd,  # Maintain hover or climb (NEVER descend)
            'distance': distance,
            'target_distance': self.target_distance,
            'tracking_active': True,
            'emergency_stop': True  # Flag for telemetry/logging
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
