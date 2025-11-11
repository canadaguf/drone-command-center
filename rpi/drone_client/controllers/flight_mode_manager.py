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
            logger.info(f"Starting gradual takeoff from {self.current_throttle}% throttle")
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
        """Update takeoff sequence.
        
        Args:
            tof_down: Downward ToF sensor reading in meters
            current_altitude: Current altitude from MAVLink (optional)
            
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
                'complete': False
            }
        
        # Detect ascent by monitoring ToF sensor
        if tof_down is not None:
            if self.last_tof_down is not None:
                # Check if distance is increasing (drone ascending)
                distance_change = tof_down - self.last_tof_down
                
                # If distance increased significantly, drone is ascending
                if distance_change > 0.02:  # 2cm increase indicates ascent
                    logger.info(f"Takeoff complete - drone ascending (ToF: {self.last_tof_down:.3f}m -> {tof_down:.3f}m)")
                    self.set_mode(FlightMode.FLYING)
                    return {
                        'throttle': self.hover_throttle,
                        'roll': 0,
                        'pitch': 0,
                        'yaw': 0,
                        'mode': 'flying',
                        'complete': True
                    }
            
            self.last_tof_down = tof_down
        
        # Gradually increase throttle
        if self.current_throttle < self.takeoff_throttle_max:
            self.current_throttle += self.takeoff_throttle_increment
            self.current_throttle = min(self.current_throttle, self.takeoff_throttle_max)
            logger.debug(f"Takeoff throttle: {self.current_throttle}%")
        
        # Safety timeout - if takeoff takes too long, assume flying
        if current_time - self.takeoff_start_time > 10.0:  # 10 second timeout
            logger.warning("Takeoff timeout - assuming flying state")
            self.set_mode(FlightMode.FLYING)
            return {
                'throttle': self.hover_throttle,
                'roll': 0,
                'pitch': 0,
                'yaw': 0,
                'mode': 'flying',
                'complete': True
            }
        
        self.last_update_time = current_time
        
        return {
            'throttle': self.current_throttle,
            'roll': 0,
            'pitch': 0,
            'yaw': 0,
            'mode': 'taking_off',
            'complete': False
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

