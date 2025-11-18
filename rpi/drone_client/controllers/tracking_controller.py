"""
Tracking controller implementing SPF-inspired algorithm.
Converts 2D person detection to 3D drone movement commands.
"""

import math
import logging
import time
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
        
        # Advanced following features (inspired by ArduPilot GSOC 2024 Visual Follow-Me)
        self.advanced_config = config.get('advanced', {})
        self.velocity_prediction = self.advanced_config.get('velocity_prediction', True)
        self.smooth_following = self.advanced_config.get('smooth_following', True)
        
        # Target velocity tracking for predictive following
        self.target_velocity_history = deque(maxlen=10)  # Store last 10 velocity estimates
        self.last_target_position = None  # (x, y, timestamp)
        
        # Target recovery mechanism for handling ID reassignment after brief dropouts
        recovery_config = config.get('recovery', {})
        self.recovery_enabled = recovery_config.get('enabled', True)
        self.recovery_timeout = recovery_config.get('timeout', 3.0)  # seconds
        self.recovery_iou_threshold = recovery_config.get('iou_threshold', 0.3)
        
        # Recovery state tracking
        self.last_target_bbox = None  # Last known bbox when target was lost
        self.recovery_start_time = None  # When recovery window started
        self.last_target_detection = None  # Last successful detection before loss
        
        logger.info(f"Tracking controller initialized: mode={self.current_distance_mode}, target_distance={self.target_distance}m")
        logger.info(f"Safety: Yaw-only mode threshold = {self.yaw_only_threshold}m")
        logger.info(f"Advanced features: velocity_prediction={self.velocity_prediction}, smooth_following={self.smooth_following}")
        logger.info(f"Recovery: enabled={self.recovery_enabled}, timeout={self.recovery_timeout}s, iou_threshold={self.recovery_iou_threshold}")
    
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
        
        # Clear recovery state when setting new target
        self.recovery_start_time = None
        self.last_target_bbox = None
        self.last_target_detection = None
        
        logger.info(f"Tracking target set: person {person_id}")
        return True
    
    def stop_tracking(self) -> None:
        """Stop tracking current target."""
        self.current_target = None
        self.tracking_active = False
        self.target_lost_time = 0
        
        # Clear recovery state when stopping tracking
        self.recovery_start_time = None
        self.last_target_bbox = None
        self.last_target_detection = None
        
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
                height_agl: Optional[float] = None) -> Dict[str, Any]:
        """Update tracking controller with new detections.
        
        Uses only visual data - no sensor dependencies.
        
        Args:
            detections: List of chair detections
            height_agl: Height above ground level in meters (from telemetry)
            
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
        
        # Store last successful detection for recovery mechanism
        self.last_target_detection = target_detection
        
        # Clear recovery state since target is found
        if self.recovery_start_time is not None:
            logger.debug("Target found - clearing recovery state")
            self.recovery_start_time = None
            self.last_target_bbox = None
        
        # Calculate tracking commands (visual-only, no sensors)
        commands = self._calculate_tracking_commands(
            target_detection, height_agl
        )
        
        # Update telemetry
        self.telemetry.update_tracking_status("TRACKING")
        
        return commands
    
    def _calculate_iou(self, bbox1: Tuple[int, int, int, int], bbox2: Tuple[int, int, int, int]) -> float:
        """Calculate IoU between two bounding boxes.
        
        Args:
            bbox1: First bounding box (x1, y1, x2, y2)
            bbox2: Second bounding box (x1, y1, x2, y2)
            
        Returns:
            IoU value (0.0 to 1.0)
        """
        x1_1, y1_1, x2_1, y2_1 = bbox1
        x1_2, y1_2, x2_2, y2_2 = bbox2
        
        # Calculate intersection
        x1_i = max(x1_1, x1_2)
        y1_i = max(y1_1, y1_2)
        x2_i = min(x2_1, x2_2)
        y2_i = min(y2_1, y2_2)
        
        if x2_i <= x1_i or y2_i <= y1_i:
            return 0.0
        
        intersection = (x2_i - x1_i) * (y2_i - y1_i)
        
        # Calculate union
        area1 = (x2_1 - x1_1) * (y2_1 - y1_1)
        area2 = (x2_2 - x1_2) * (y2_2 - y1_2)
        union = area1 + area2 - intersection
        
        if union <= 0:
            return 0.0
        
        return intersection / union
    
    def _find_target_detection(self, detections: List[Dict[str, Any]]) -> Optional[Dict[str, Any]]:
        """Find target person in detections.
        
        Implements recovery mechanism: if target ID not found but recovery is active,
        attempts to match new detections by IoU with last known bbox.
        
        Args:
            detections: List of person detections
            
        Returns:
            Target detection if found, None otherwise
        """
        # First, try to find target by ID (normal case)
        for detection in detections:
            if detection.get('id') == self.current_target:
                # Target found - clear recovery state
                self.recovery_start_time = None
                self.last_target_bbox = None
                return detection
        
        # Target not found by ID - check if recovery is enabled and active
        if not self.recovery_enabled or self.last_target_bbox is None:
            return None
        
        # Check if recovery window is still active
        current_time = time.time()
        if self.recovery_start_time is None:
            # Recovery window hasn't started yet - shouldn't happen, but handle gracefully
            return None
        
        recovery_duration = current_time - self.recovery_start_time
        if recovery_duration > self.recovery_timeout:
            # Recovery window expired - give up
            logger.debug(f"Recovery window expired ({recovery_duration:.2f}s > {self.recovery_timeout}s)")
            self.recovery_start_time = None
            self.last_target_bbox = None
            return None
        
        # Recovery window active - try to match by IoU
        best_match = None
        best_iou = 0.0
        
        for detection in detections:
            detection_bbox = detection.get('bbox')
            if detection_bbox is None:
                continue
            
            # Calculate IoU with last known bbox
            iou = self._calculate_iou(self.last_target_bbox, detection_bbox)
            
            if iou > best_iou and iou >= self.recovery_iou_threshold:
                best_iou = iou
                best_match = detection
        
        if best_match is not None:
            # Found a match - automatically reassign target ID
            new_target_id = best_match.get('id')
            logger.info(f"Target recovered: reassigning from ID {self.current_target} to ID {new_target_id} "
                       f"(IoU={best_iou:.3f}, recovery_duration={recovery_duration:.2f}s)")
            self.current_target = new_target_id
            self.recovery_start_time = None
            self.last_target_bbox = None
            return best_match
        
        # No match found yet, but recovery window still active
        logger.debug(f"Recovery attempt: no match found (best IoU={best_iou:.3f}, "
                    f"threshold={self.recovery_iou_threshold}, {recovery_duration:.2f}s remaining)")
        return None
    
    def _handle_lost_target(self) -> Dict[str, Any]:
        """Handle lost target situation.
        
        Stores last known bbox and starts recovery timer if recovery is enabled.
        
        Returns:
            Dictionary containing appropriate commands
        """
        # Store last known bbox and start recovery timer if not already started
        if self.recovery_enabled and self.last_target_detection is not None:
            if self.recovery_start_time is None:
                # First time target is lost - store bbox and start recovery window
                self.last_target_bbox = self.last_target_detection.get('bbox')
                self.recovery_start_time = time.time()
                logger.debug(f"Target lost - starting recovery window (bbox={self.last_target_bbox})")
        
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
            # Land - recovery window expired or not enabled
            if self.recovery_start_time is not None:
                logger.info("Recovery window expired - giving up on target recovery")
                self.recovery_start_time = None
                self.last_target_bbox = None
            self.telemetry.update_tracking_status("LANDING")
            return self._get_land_commands()
    
    def _calculate_tracking_commands(self, detection: Dict[str, Any], 
                                   height_agl: Optional[float] = None) -> Dict[str, Any]:
        """Calculate tracking commands from detection using advanced visual tracking.
        
        Implements ArduPilot GSOC 2024 Visual Follow-Me inspired algorithms:
        - Velocity-based predictive following
        - Smooth following with oscillation reduction
        
        Args:
            detection: Target detection
            height_agl: Height above ground level in meters (from telemetry)
            
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
        
        # Convert to angular errors (fixed camera, no gimbal compensation needed)
        fov_h_rad = math.radians(self.fov_horizontal)
        fov_v_rad = math.radians(self.fov_vertical)
        
        angle_yaw = (error_x / self.camera_width) * fov_h_rad
        angle_pitch = (error_y / self.camera_height) * fov_v_rad
        
        # Advanced: Velocity-based prediction for smoother following
        if self.velocity_prediction:
            predicted_error_x, predicted_error_y = self._predict_target_movement(
                center_x, center_y, error_x, error_y
            )
            # Blend current error with predicted error (70% current, 30% predicted)
            error_x = 0.7 * error_x + 0.3 * predicted_error_x
            error_y = 0.7 * error_y + 0.3 * predicted_error_y
            
            # Recalculate angles after error blending
            angle_yaw = (error_x / self.camera_width) * fov_h_rad
            angle_pitch = (error_y / self.camera_height) * fov_v_rad
        
        # Determine distance using visual estimation only (no sensors)
        distance = self._estimate_distance(detection)
        
        # Advanced: Update target velocity history for predictive following
        if self.velocity_prediction:
            self._update_target_velocity(center_x, center_y, distance)
        
        # SAFETY: Yaw-only mode for close targets (SPF-inspired)
        # Prevents forward movement when target is too close to avoid collisions
        # Uses visual distance estimation only
        if distance < self.yaw_only_threshold:
            logger.warning(f"Target too close ({distance:.2f}m < {self.yaw_only_threshold}m) - YAW ONLY mode")
            
            # Only calculate yaw correction
            yaw_error = angle_yaw
            pid_outputs = self.pid_manager.update({'yaw': yaw_error})
            yaw_cmd = int(np.clip(pid_outputs['yaw'], -100, 100))
            
            # Calculate throttle dynamically based on altitude (AGL height)
            throttle_cmd = self._calculate_throttle(
                height_agl, error_y
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
        
        # Advanced: Smooth following - apply smoothing filter to reduce oscillations
        if self.smooth_following:
            lateral_x, lateral_y, angle_yaw = self._apply_smoothing(
                lateral_x, lateral_y, angle_yaw
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
        
        # Calculate throttle dynamically based on altitude (AGL height)
        throttle_cmd = self._calculate_throttle(
            height_agl, error_y
        )
        
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
    
    def _estimate_distance(self, detection: Dict[str, Any]) -> float:
        """Estimate distance to target using visual data only (SPF-inspired).
        
        Uses bbox size for distance estimation - no sensor dependencies.
        
        Args:
            detection: Target detection
            
        Returns:
            Estimated distance in meters
        """
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
    
    def _calculate_throttle(self, height_agl: Optional[float],
                           vertical_error: float = 0.0) -> int:
        """Calculate throttle dynamically based on height above ground level (AGL).
        
        Uses PID control to maintain target altitude using AGL height from flight controller.
        Always maintains hover throttle for stabilization - never goes below base_hover_throttle.
        
        Args:
            height_agl: Height above ground level in meters (from telemetry)
            vertical_error: Vertical pixel error from visual tracking
            
        Returns:
            Throttle command (0-100) - always maintains minimum hover throttle
        """
        # Use AGL height (height above ground level)
        actual_height = height_agl
        
        # If no height data available, use base hover throttle for stabilization
        if actual_height is None:
            logger.debug("No height AGL data available - using base hover throttle for stabilization")
            return self.base_hover_throttle
        
        # Calculate altitude error (target - actual height AGL)
        altitude_error = self.target_altitude - actual_height
        
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
        
        # Apply safety limits based on height AGL
        # Always maintain minimum hover throttle for stabilization
        if actual_height < self.min_height:
            # Low altitude - ensure minimum climb
            logger.debug(f"Low height AGL ({actual_height:.2f}m) - ensuring minimum climb")
            throttle = max(throttle, self.base_hover_throttle + 15)
        elif actual_height > self.max_height:
            # Too high - limit maximum throttle but maintain hover
            throttle = min(throttle, self.base_hover_throttle - 10)
        
        # CRITICAL: Always maintain minimum hover throttle for stabilization
        # Never go below base_hover_throttle to prevent falling
        throttle = max(throttle, self.base_hover_throttle)
        
        # Clamp to valid range (0-100)
        throttle = int(np.clip(throttle, 0, 100))
        
        logger.debug(f"Throttle: height_agl={actual_height:.2f}m, target={self.target_altitude:.2f}m, "
                    f"error={altitude_error:.2f}m, throttle={throttle}")
        
        return throttle
    
    
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
                                     height_agl: Optional[float] = None) -> Dict[str, Any]:
        """Get emergency stop commands when target too close.
        
        CRITICAL: Always maintains hover throttle to prevent falling.
        Uses visual distance estimation only - no sensors.
        
        Args:
            distance: Current estimated distance to target (visual)
            height_agl: Height above ground level in meters (from telemetry)
            
        Returns:
            Dictionary containing emergency stop commands
        """
        # SAFETY: Emergency stop MUST maintain altitude to prevent crash
        # Use dynamic throttle calculation, but ensure minimum hover throttle
        throttle_cmd = self._calculate_throttle(height_agl, 0.0)
        
        # Emergency stop: ensure we never go below hover throttle
        throttle_cmd = max(throttle_cmd, self.base_hover_throttle)
        
        # If height too low, force climb
        if height_agl is not None and height_agl < self.min_height:
            logger.warning(f"Emergency stop: Height AGL too low ({height_agl:.2f}m) - forcing climb")
            throttle_cmd = max(throttle_cmd, self.base_hover_throttle + 30)
        
        logger.info(f"Emergency stop: Maintaining altitude (throttle={throttle_cmd}, height_agl={height_agl})")
        
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
        recovery_active = False
        recovery_time_remaining = 0.0
        
        if self.recovery_start_time is not None:
            recovery_active = True
            recovery_duration = time.time() - self.recovery_start_time
            recovery_time_remaining = max(0.0, self.recovery_timeout - recovery_duration)
        
        return {
            'tracking_active': self.tracking_active,
            'current_target': self.current_target,
            'distance_mode': self.current_distance_mode,
            'target_distance': self.target_distance,
            'target_lost_time': self.target_lost_time,
            'hover_timeout': self.hover_timeout,
            'land_timeout': self.land_timeout,
            'auto_adjust_altitude': self.auto_adjust_altitude,
            'recovery_enabled': self.recovery_enabled,
            'recovery_active': recovery_active,
            'recovery_time_remaining': recovery_time_remaining,
            'recovery_timeout': self.recovery_timeout,
            'recovery_iou_threshold': self.recovery_iou_threshold
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
    
    def _predict_target_movement(self, center_x: float, center_y: float,
                                 error_x: float, error_y: float) -> Tuple[float, float]:
        """Predict target movement based on velocity history.
        
        Implements velocity-based prediction for smoother following.
        
        Args:
            center_x: Current target center X coordinate
            center_y: Current target center Y coordinate
            error_x: Current horizontal error
            error_y: Current vertical error
            
        Returns:
            Tuple of (predicted_error_x, predicted_error_y)
        """
        current_time = time.time()
        
        if self.last_target_position is None:
            self.last_target_position = (center_x, center_y, current_time)
            return (error_x, error_y)
        
        last_x, last_y, last_time = self.last_target_position
        dt = current_time - last_time
        
        if dt < 0.01:  # Too small time difference
            return (error_x, error_y)
        
        # Calculate velocity in pixels per second
        vx = (center_x - last_x) / dt
        vy = (center_y - last_y) / dt
        
        # Store velocity
        self.target_velocity_history.append((vx, vy, current_time))
        
        # Predict future position (predict 0.1 seconds ahead)
        prediction_time = 0.1
        predicted_x = center_x + vx * prediction_time
        predicted_y = center_y + vy * prediction_time
        
        # Calculate predicted error
        image_center_x = self.camera_width / 2
        image_center_y = self.camera_height / 2
        predicted_error_x = predicted_x - image_center_x
        predicted_error_y = predicted_y - image_center_y
        
        # Update last position
        self.last_target_position = (center_x, center_y, current_time)
        
        return (predicted_error_x, predicted_error_y)
    
    def _update_target_velocity(self, center_x: float, center_y: float, distance: float) -> None:
        """Update target velocity tracking.
        
        Args:
            center_x: Current target center X coordinate
            center_y: Current target center Y coordinate
            distance: Current estimated distance to target
        """
        current_time = time.time()
        
        if self.last_target_position is None:
            self.last_target_position = (center_x, center_y, current_time)
            return
        
        last_x, last_y, last_time = self.last_target_position
        dt = current_time - last_time
        
        if dt < 0.01:  # Too small time difference
            return
        
        # Calculate velocity in pixels per second
        vx = (center_x - last_x) / dt
        vy = (center_y - last_y) / dt
        
        # Store velocity (already done in _predict_target_movement, but keep for future use)
        self.last_target_position = (center_x, center_y, current_time)
    
    def _apply_smoothing(self, lateral_x: float, lateral_y: float, 
                        angle_yaw: float) -> Tuple[float, float, float]:
        """Apply smoothing filter to reduce oscillations.
        
        Implements exponential moving average for smoother following.
        
        Args:
            lateral_x: Lateral X displacement
            lateral_y: Lateral Y displacement
            angle_yaw: Yaw angle error
            
        Returns:
            Tuple of (smoothed_lateral_x, smoothed_lateral_y, smoothed_angle_yaw)
        """
        # Exponential moving average with alpha = 0.7 (30% new, 70% old)
        alpha = 0.7
        
        # Store smoothed values (initialize if needed)
        if not hasattr(self, '_smoothed_lateral_x'):
            self._smoothed_lateral_x = lateral_x
            self._smoothed_lateral_y = lateral_y
            self._smoothed_angle_yaw = angle_yaw
            return (lateral_x, lateral_y, angle_yaw)
        
        # Apply smoothing
        self._smoothed_lateral_x = alpha * lateral_x + (1 - alpha) * self._smoothed_lateral_x
        self._smoothed_lateral_y = alpha * lateral_y + (1 - alpha) * self._smoothed_lateral_y
        self._smoothed_angle_yaw = alpha * angle_yaw + (1 - alpha) * self._smoothed_angle_yaw
        
        return (self._smoothed_lateral_x, self._smoothed_lateral_y, self._smoothed_angle_yaw)
