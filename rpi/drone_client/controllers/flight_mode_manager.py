"""
Flight mode manager for gradual takeoff/landing and stabilization.
Handles state transitions and ToF sensor-based landing detection.
"""

import logging
import time
from enum import Enum
from typing import Dict, Any, Optional, Callable

logger = logging.getLogger(__name__)


class FlightMode(Enum):
    """Flight mode states."""
    IDLE = "idle"
    ARMING = "arming"
    TAKING_OFF = "taking_off"
    FLYING = "flying"
    LANDING = "landing"
    LANDED = "landed"
    LOITERING = "loitering"


class FlightModeManager:
    """Manages flight modes with gradual takeoff/landing using ToF sensors."""
    
    def __init__(self, config: Dict[str, Any]):
        """Initialize flight mode manager.
        
        Args:
            config: Flight mode configuration
        """
        self.config = config
        
        # Flight mode state
        self.current_mode = FlightMode.IDLE
        
        # Takeoff configuration
        self.takeoff_config = config.get('takeoff', {})
        self.takeoff_throttle_start = self.takeoff_config.get('throttle_start', 40)
        self.takeoff_throttle_increment = self.takeoff_config.get('throttle_increment', 2)
        self.takeoff_throttle_max = self.takeoff_config.get('throttle_max', 70)
        self.takeoff_update_interval = self.takeoff_config.get('update_interval', 0.1)  # seconds
        self.takeoff_target_height = self.takeoff_config.get('target_height', 0.5)  # Target height in meters
        self.takeoff_height_tolerance = self.takeoff_config.get('height_tolerance', 0.05)  # 5cm tolerance
        self.takeoff_kp = self.takeoff_config.get('kp', 15.0)  # Proportional gain for height control
        
        # Landing configuration
        self.landing_config = config.get('landing', {})
        self.landing_throttle_decrement = self.landing_config.get('throttle_decrement', 2)
        self.landing_detection_distance = self.landing_config.get('detection_distance', 0.06)  # 6cm
        self.landing_detection_tolerance = self.landing_config.get('detection_tolerance', 0.006)  # 10% of 6cm
        self.landing_update_interval = self.landing_config.get('update_interval', 0.1)  # seconds
        
        # Stabilization configuration
        self.stabilization_config = config.get('stabilization', {})
        self.hover_throttle = self.stabilization_config.get('hover_throttle', 50)
        self.stabilization_update_interval = self.stabilization_config.get('update_interval', 0.1)  # seconds
        
        # State tracking
        self.current_throttle = 0
        self.last_tof_down = None
        self.last_tof_time = 0
        self.takeoff_start_time = 0
        self.takeoff_initial_tof = None  # Initial ToF reading when takeoff starts
        self.landing_start_time = 0
        self.last_update_time = 0
        
        # Callbacks
        self.on_mode_change_callback = None
        self.on_landed_callback = None
        
        logger.info(f"Flight mode manager initialized: hover_throttle={self.hover_throttle}%")
    
    def set_mode(self, mode: FlightMode) -> bool:
        """Set flight mode.
        
        Args:
            mode: Target flight mode
            
        Returns:
            True if mode change successful, False otherwise
        """
        if mode == self.current_mode:
            return True
        
        old_mode = self.current_mode
        self.current_mode = mode
        
        # Reset state when entering new modes
        if mode == FlightMode.TAKING_OFF:
            self.current_throttle = self.takeoff_throttle_start
            self.takeoff_start_time = time.time()
            self.last_tof_down = None
            self.takeoff_initial_tof = None  # Will be set on first ToF reading
            logger.info(f"Starting gradual takeoff - target height: {self.takeoff_target_height}m")
        elif mode == FlightMode.LANDING:
            self.landing_start_time = time.time()
            self.last_tof_down = None
            logger.info("Starting gradual landing")
        elif mode == FlightMode.LANDED:
            self.current_throttle = 0
            logger.info("Drone landed")
        elif mode == FlightMode.LOITERING:
            self.current_throttle = self.hover_throttle
            logger.info("Entering loiter mode")
        
        if self.on_mode_change_callback:
            self.on_mode_change_callback(old_mode, mode)
        
        logger.info(f"Flight mode changed: {old_mode.value} -> {mode.value}")
        return True
    
    def get_mode(self) -> FlightMode:
        """Get current flight mode.
        
        Returns:
            Current flight mode
        """
        return self.current_mode
    
    def update_takeoff(self, tof_down: Optional[float], 
                      current_altitude: Optional[float] = None) -> Dict[str, Any]:
        """Update takeoff sequence with dynamic throttle adjustment based on ToF readings.
        
        Args:
            tof_down: Downward ToF sensor reading in meters (distance to ground)
            current_altitude: Current altitude from MAVLink (optional, for reference)
            
        Returns:
            Dictionary with throttle command and status
        """
        current_time = time.time()
        
        # Check if enough time has passed since last update
        if current_time - self.last_update_time < self.takeoff_update_interval:
            return {
                'throttle': self.current_throttle,
                'roll': 0,
                'pitch': 0,
                'yaw': 0,
                'mode': 'taking_off',
                'complete': False,
                'current_height': None
            }
        
        # Store initial ToF reading when takeoff starts (ground distance)
        if tof_down is not None and self.takeoff_initial_tof is None:
            self.takeoff_initial_tof = tof_down
            logger.info(f"Takeoff initialized - ground distance: {tof_down:.3f}m, target height: {self.takeoff_target_height}m")
        
        # Calculate current height from ToF sensor
        current_height = None
        if tof_down is not None and self.takeoff_initial_tof is not None:
            # Height = current ToF distance - initial ground distance
            # As drone rises, ToF increases, so height = tof_down - initial_tof
            current_height = tof_down - self.takeoff_initial_tof
            
            # Check if target height reached (within tolerance)
            height_error = self.takeoff_target_height - current_height
            if abs(height_error) <= self.takeoff_height_tolerance:
                logger.info(f"Takeoff complete - reached target height: {current_height:.3f}m (target: {self.takeoff_target_height}m)")
                self.set_mode(FlightMode.FLYING)
                return {
                    'throttle': self.hover_throttle,
                    'roll': 0,
                    'pitch': 0,
                    'yaw': 0,
                    'mode': 'flying',
                    'complete': True,
                    'current_height': current_height
                }
            
            # Dynamic throttle adjustment based on height error
            # Use proportional control: throttle = base_throttle + Kp * height_error
            # Positive height_error means we're below target, need more throttle
            # Negative height_error means we're above target, need less throttle
            
            # Base throttle for takeoff (minimum to maintain lift)
            base_throttle = self.takeoff_throttle_start
            
            # Proportional control: adjust throttle based on height error
            throttle_adjustment = self.takeoff_kp * height_error
            
            # Calculate new throttle
            new_throttle = base_throttle + throttle_adjustment
            
            # Clamp throttle to safe limits
            new_throttle = max(self.takeoff_throttle_start, min(new_throttle, self.takeoff_throttle_max))
            
            # Smooth throttle changes (limit rate of change)
            throttle_change = new_throttle - self.current_throttle
            max_throttle_change = self.takeoff_throttle_increment * 2  # Allow faster changes during takeoff
            if abs(throttle_change) > max_throttle_change:
                if throttle_change > 0:
                    self.current_throttle += max_throttle_change
                else:
                    self.current_throttle -= max_throttle_change
            else:
                self.current_throttle = new_throttle
            
            # Clamp final throttle
            self.current_throttle = max(self.takeoff_throttle_start, min(self.current_throttle, self.takeoff_throttle_max))
            
            logger.debug(f"Takeoff: height={current_height:.3f}m, target={self.takeoff_target_height}m, "
                        f"error={height_error:.3f}m, throttle={self.current_throttle:.1f}%")
            
            self.last_tof_down = tof_down
        elif tof_down is None:
            # No ToF reading available - use fallback incremental approach
            logger.warning("No ToF reading available - using fallback incremental throttle")
            if self.current_throttle < self.takeoff_throttle_max:
                self.current_throttle += self.takeoff_throttle_increment
                self.current_throttle = min(self.current_throttle, self.takeoff_throttle_max)
        
        # Safety timeout - if takeoff takes too long, assume flying
        if current_time - self.takeoff_start_time > 15.0:  # 15 second timeout
            logger.warning("Takeoff timeout - assuming flying state")
            self.set_mode(FlightMode.FLYING)
            return {
                'throttle': self.hover_throttle,
                'roll': 0,
                'pitch': 0,
                'yaw': 0,
                'mode': 'flying',
                'complete': True,
                'current_height': current_height
            }
        
        self.last_update_time = current_time
        
        return {
            'throttle': self.current_throttle,
            'roll': 0,
            'pitch': 0,
            'yaw': 0,
            'mode': 'taking_off',
            'complete': False,
            'current_height': current_height
        }
    
    def update_landing(self, tof_down: Optional[float],
                      current_altitude: Optional[float] = None) -> Dict[str, Any]:
        """Update landing sequence.
        
        Args:
            tof_down: Downward ToF sensor reading in meters
            current_altitude: Current altitude from MAVLink (optional)
            
        Returns:
            Dictionary with throttle command, status, and landed flag
        """
        current_time = time.time()
        
        # Check if enough time has passed since last update
        if current_time - self.last_update_time < self.landing_update_interval:
            return {
                'throttle': self.current_throttle,
                'roll': 0,
                'pitch': 0,
                'yaw': 0,
                'mode': 'landing',
                'complete': False,
                'landed': False
            }
        
        # Check if landed using ToF sensor
        if tof_down is not None:
            # Check if within landing detection range (6cm ± 10% = 5.4cm to 6.6cm)
            landing_min = self.landing_detection_distance - self.landing_detection_tolerance
            landing_max = self.landing_detection_distance + self.landing_detection_tolerance
            
            if landing_min <= tof_down <= landing_max:
                logger.info(f"Landing detected - ToF reading: {tof_down:.3f}m (target: {self.landing_detection_distance:.3f}m)")
                self.set_mode(FlightMode.LANDED)
                
                if self.on_landed_callback:
                    self.on_landed_callback()
                
                return {
                    'throttle': 0,
                    'roll': 0,
                    'pitch': 0,
                    'yaw': 0,
                    'mode': 'landed',
                    'complete': True,
                    'landed': True
                }
        
        # Gradually decrease throttle, but never below minimum hover throttle
        # until we're very close to ground
        if tof_down is not None and tof_down > 0.15:  # Above 15cm
            # Reduce throttle gradually
            if self.current_throttle > self.hover_throttle:
                self.current_throttle -= self.landing_throttle_decrement
                self.current_throttle = max(self.current_throttle, self.hover_throttle)
                logger.debug(f"Landing throttle: {self.current_throttle}% (ToF: {tof_down:.3f}m)")
        elif tof_down is not None and tof_down > 0.10:  # Between 10-15cm
            # More aggressive descent
            if self.current_throttle > self.hover_throttle - 10:
                self.current_throttle -= self.landing_throttle_decrement * 1.5
                self.current_throttle = max(self.current_throttle, self.hover_throttle - 10)
                logger.debug(f"Landing throttle: {self.current_throttle}% (ToF: {tof_down:.3f}m)")
        elif tof_down is not None:  # Below 10cm
            # Very close to ground, reduce throttle more aggressively
            if self.current_throttle > 20:
                self.current_throttle -= self.landing_throttle_decrement * 2
                self.current_throttle = max(self.current_throttle, 20)
                logger.debug(f"Landing throttle: {self.current_throttle}% (ToF: {tof_down:.3f}m)")
        else:
            # No ToF reading - conservative descent
            if self.current_throttle > self.hover_throttle:
                self.current_throttle -= self.landing_throttle_decrement
                self.current_throttle = max(self.current_throttle, self.hover_throttle)
        
        self.last_update_time = current_time
        
        return {
            'throttle': self.current_throttle,
            'roll': 0,
            'pitch': 0,
            'yaw': 0,
            'mode': 'landing',
            'complete': False,
            'landed': False
        }
    
    def get_stabilization_commands(self) -> Dict[str, Any]:
        """Get stabilization commands (zero roll/pitch, hover throttle).
        
        Returns:
            Dictionary with stabilization commands
        """
        return {
            'roll': 0,
            'pitch': 0,
            'yaw': 0,
            'throttle': self.hover_throttle,
            'mode': 'stabilization'
        }
    
    def get_loiter_commands(self) -> Dict[str, Any]:
        """Get loiter mode commands (zero roll/pitch/yaw, hover throttle).
        
        Returns:
            Dictionary with loiter commands
        """
        return {
            'roll': 0,
            'pitch': 0,
            'yaw': 0,
            'throttle': self.hover_throttle,
            'mode': 'loitering'
        }
    
    def set_callbacks(self, on_mode_change: Optional[Callable] = None,
                     on_landed: Optional[Callable] = None) -> None:
        """Set callback functions.
        
        Args:
            on_mode_change: Called when flight mode changes (old_mode, new_mode)
            on_landed: Called when landing is detected
        """
        self.on_mode_change_callback = on_mode_change
        self.on_landed_callback = on_landed
    
    def reset(self) -> None:
        """Reset flight mode manager to idle state."""
        self.set_mode(FlightMode.IDLE)
        self.current_throttle = 0
        self.last_tof_down = None
        self.last_tof_time = 0
        self.takeoff_initial_tof = None
        logger.info("Flight mode manager reset")
    
    def get_status(self) -> Dict[str, Any]:
        """Get flight mode manager status.
        
        Returns:
            Dictionary containing status information
        """
        return {
            'mode': self.current_mode.value,
            'current_throttle': self.current_throttle,
            'hover_throttle': self.hover_throttle,
            'last_tof_down': self.last_tof_down
        }

