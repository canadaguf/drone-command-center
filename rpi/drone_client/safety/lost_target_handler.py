"""
Lost target handler for managing target loss scenarios.
Implements hover→land sequence when target is lost.
"""

import logging
import time
from typing import Dict, Any, Optional, Callable
from enum import Enum

logger = logging.getLogger(__name__)

class TargetStatus(Enum):
    """Target tracking status."""
    TRACKING = "TRACKING"
    LOST = "LOST"
    HOVERING = "HOVERING"
    LANDING = "LANDING"

class LostTargetHandler:
    """Handler for lost target scenarios."""
    
    def __init__(self, config: Dict[str, Any]):
        """Initialize lost target handler.
        
        Args:
            config: Lost target configuration
        """
        self.hover_timeout = config.get('hover_timeout', 2.0)
        self.land_timeout = config.get('land_timeout', 10.0)
        
        # State
        self.status = TargetStatus.TRACKING
        self.target_lost_time = 0.0
        self.last_target_time = 0.0
        self.hover_start_time = 0.0
        self.land_start_time = 0.0
        
        # Callbacks
        self.on_status_change_callback = None
        self.on_hover_callback = None
        self.on_land_callback = None
        
        logger.info(f"Lost target handler initialized: hover_timeout={self.hover_timeout}s, land_timeout={self.land_timeout}s")
    
    def set_callbacks(self, on_status_change: Optional[Callable] = None,
                     on_hover: Optional[Callable] = None,
                     on_land: Optional[Callable] = None) -> None:
        """Set callback functions.
        
        Args:
            on_status_change: Called when status changes
            on_hover: Called when entering hover mode
            on_land: Called when entering land mode
        """
        self.on_status_change_callback = on_status_change
        self.on_hover_callback = on_hover
        self.on_land_callback = on_land
    
    def update(self, target_detected: bool) -> TargetStatus:
        """Update target tracking status.
        
        Args:
            target_detected: Whether target is currently detected
            
        Returns:
            Current target status
        """
        current_time = time.time()
        
        if target_detected:
            # Target found
            if self.status != TargetStatus.TRACKING:
                self._change_status(TargetStatus.TRACKING)
            
            self.last_target_time = current_time
            self.target_lost_time = 0.0
            self.hover_start_time = 0.0
            self.land_start_time = 0.0
            
        else:
            # Target lost
            if self.status == TargetStatus.TRACKING:
                # Just lost target
                self._change_status(TargetStatus.LOST)
                self.target_lost_time = current_time
                
            elif self.status == TargetStatus.LOST:
                # Check if should start hovering
                lost_duration = current_time - self.target_lost_time
                if lost_duration >= self.hover_timeout:
                    self._change_status(TargetStatus.HOVERING)
                    self.hover_start_time = current_time
                    
                    if self.on_hover_callback:
                        self.on_hover_callback()
                    
                    logger.info("Target lost - entering hover mode")
                
            elif self.status == TargetStatus.HOVERING:
                # Check if should start landing
                hover_duration = current_time - self.hover_start_time
                total_lost_duration = current_time - self.target_lost_time
                
                if total_lost_duration >= self.land_timeout:
                    self._change_status(TargetStatus.LANDING)
                    self.land_start_time = current_time
                    
                    if self.on_land_callback:
                        self.on_land_callback()
                    
                    logger.warning("Target lost too long - initiating landing")
        
        return self.status
    
    def _change_status(self, new_status: TargetStatus) -> None:
        """Change target status.
        
        Args:
            new_status: New target status
        """
        old_status = self.status
        self.status = new_status
        
        if self.on_status_change_callback:
            self.on_status_change_callback(old_status, new_status)
        
        logger.info(f"Target status changed: {old_status.value} → {new_status.value}")
    
    def get_status(self) -> TargetStatus:
        """Get current target status.
        
        Returns:
            Current target status
        """
        return self.status
    
    def get_status_string(self) -> str:
        """Get current status as string.
        
        Returns:
            Status string
        """
        return self.status.value
    
    def get_lost_duration(self) -> float:
        """Get duration target has been lost.
        
        Returns:
            Duration in seconds, 0 if target is found
        """
        if self.status == TargetStatus.TRACKING:
            return 0.0
        
        return time.time() - self.target_lost_time
    
    def get_hover_duration(self) -> float:
        """Get duration in hover mode.
        
        Returns:
            Duration in seconds, 0 if not hovering
        """
        if self.status not in [TargetStatus.HOVERING, TargetStatus.LANDING]:
            return 0.0
        
        return time.time() - self.hover_start_time
    
    def get_land_duration(self) -> float:
        """Get duration in landing mode.
        
        Returns:
            Duration in seconds, 0 if not landing
        """
        if self.status != TargetStatus.LANDING:
            return 0.0
        
        return time.time() - self.land_start_time
    
    def get_time_until_hover(self) -> float:
        """Get time until hover mode starts.
        
        Returns:
            Time in seconds, 0 if already hovering or landing
        """
        if self.status != TargetStatus.LOST:
            return 0.0
        
        lost_duration = time.time() - self.target_lost_time
        return max(0.0, self.hover_timeout - lost_duration)
    
    def get_time_until_land(self) -> float:
        """Get time until landing starts.
        
        Returns:
            Time in seconds, 0 if already landing
        """
        if self.status == TargetStatus.LANDING:
            return 0.0
        
        total_lost_duration = time.time() - self.target_lost_time
        return max(0.0, self.land_timeout - total_lost_duration)
    
    def reset(self) -> None:
        """Reset handler state."""
        self.status = TargetStatus.TRACKING
        self.target_lost_time = 0.0
        self.last_target_time = 0.0
        self.hover_start_time = 0.0
        self.land_start_time = 0.0
        
        logger.info("Lost target handler reset")
    
    def force_hover(self) -> None:
        """Force hover mode."""
        self._change_status(TargetStatus.HOVERING)
        self.hover_start_time = time.time()
        logger.info("Forced hover mode")
    
    def force_land(self) -> None:
        """Force landing mode."""
        self._change_status(TargetStatus.LANDING)
        self.land_start_time = time.time()
        logger.info("Forced landing mode")
    
    def get_config(self) -> Dict[str, Any]:
        """Get handler configuration.
        
        Returns:
            Dictionary containing configuration
        """
        return {
            'hover_timeout': self.hover_timeout,
            'land_timeout': self.land_timeout
        }
    
    def set_timeouts(self, hover_timeout: float, land_timeout: float) -> None:
        """Set timeout values.
        
        Args:
            hover_timeout: Hover timeout in seconds
            land_timeout: Land timeout in seconds
        """
        self.hover_timeout = hover_timeout
        self.land_timeout = land_timeout
        logger.info(f"Timeouts updated: hover={hover_timeout}s, land={land_timeout}s")
    
    def get_status_info(self) -> Dict[str, Any]:
        """Get detailed status information.
        
        Returns:
            Dictionary containing status info
        """
        return {
            'status': self.status.value,
            'lost_duration': self.get_lost_duration(),
            'hover_duration': self.get_hover_duration(),
            'land_duration': self.get_land_duration(),
            'time_until_hover': self.get_time_until_hover(),
            'time_until_land': self.get_time_until_land(),
            'target_lost_time': self.target_lost_time,
            'last_target_time': self.last_target_time
        }
