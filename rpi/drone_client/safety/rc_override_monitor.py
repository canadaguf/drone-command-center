"""
RC override monitor for detecting manual control takeover.
Monitors RC input activity and provides safety notifications.
"""

import logging
from typing import Dict, Any, Optional, Callable
from dataclasses import dataclass
import time

logger = logging.getLogger(__name__)

@dataclass
class RCOverrideState:
    """RC override state information."""
    active: bool = False
    last_activity: float = 0.0
    channel_changes: Dict[int, int] = None
    neutral_time: float = 0.0
    
    def __post_init__(self):
        if self.channel_changes is None:
            self.channel_changes = {}

class RCOverrideMonitor:
    """Monitor for RC override detection."""
    
    def __init__(self, config: Dict[str, Any]):
        """Initialize RC override monitor.
        
        Args:
            config: RC override configuration
        """
        self.enabled = config.get('detection_enabled', True)
        self.threshold = config.get('threshold', 100)
        self.neutral_timeout = config.get('neutral_timeout', 2.0)
        
        # State
        self.state = RCOverrideState()
        self.last_rc_channels = None
        self.override_start_time = 0.0
        
        # Callbacks
        self.on_override_start_callback = None
        self.on_override_end_callback = None
        
        logger.info(f"RC override monitor initialized: enabled={self.enabled}, threshold={self.threshold}")
    
    def set_callbacks(self, on_start: Optional[Callable] = None,
                     on_end: Optional[Callable] = None) -> None:
        """Set callback functions.
        
        Args:
            on_start: Called when override starts
            on_end: Called when override ends
        """
        self.on_override_start_callback = on_start
        self.on_override_end_callback = on_end
    
    def update(self, rc_channels: Optional[Dict[str, int]]) -> bool:
        """Update RC override detection.
        
        Args:
            rc_channels: Current RC channel values
            
        Returns:
            True if override is active, False otherwise
        """
        if not self.enabled or rc_channels is None:
            return False
        
        current_time = time.time()
        override_detected = False
        
        # Check for significant channel changes
        if self.last_rc_channels is not None:
            for channel, value in rc_channels.items():
                if channel in self.last_rc_channels:
                    change = abs(value - self.last_rc_channels[channel])
                    if change > self.threshold:
                        override_detected = True
                        self.state.channel_changes[channel] = change
                        break
        
        # Update state
        if override_detected:
            if not self.state.active:
                # Override just started
                self.state.active = True
                self.state.last_activity = current_time
                self.override_start_time = current_time
                
                if self.on_override_start_callback:
                    self.on_override_start_callback()
                
                logger.info("RC override detected - manual control active")
        else:
            # Check if all channels are neutral
            all_neutral = self._check_neutral_channels(rc_channels)
            
            if all_neutral:
                if self.state.active:
                    # Check if neutral for long enough
                    neutral_duration = current_time - self.state.last_activity
                    if neutral_duration >= self.neutral_timeout:
                        # Override ended
                        self.state.active = False
                        self.state.neutral_time = current_time
                        
                        if self.on_override_end_callback:
                            self.on_override_end_callback()
                        
                        logger.info("RC override ended - returning to autonomous")
            else:
                # Update last activity time
                self.state.last_activity = current_time
        
        # Update last channels
        self.last_rc_channels = rc_channels.copy()
        
        return self.state.active
    
    def _check_neutral_channels(self, rc_channels: Dict[str, int]) -> bool:
        """Check if all channels are near neutral.
        
        Args:
            rc_channels: Current RC channel values
            
        Returns:
            True if all channels are neutral, False otherwise
        """
        neutral_range = 50  # ±50 from 1500
        neutral_center = 1500
        
        for value in rc_channels.values():
            if abs(value - neutral_center) > neutral_range:
                return False
        
        return True
    
    def is_override_active(self) -> bool:
        """Check if RC override is currently active.
        
        Returns:
            True if override is active, False otherwise
        """
        return self.state.active
    
    def get_override_duration(self) -> float:
        """Get duration of current override.
        
        Returns:
            Duration in seconds, 0 if not active
        """
        if not self.state.active:
            return 0.0
        
        return time.time() - self.override_start_time
    
    def get_channel_changes(self) -> Dict[int, int]:
        """Get channel change information.
        
        Returns:
            Dictionary of channel changes
        """
        return self.state.channel_changes.copy()
    
    def reset(self) -> None:
        """Reset override monitor state."""
        self.state = RCOverrideState()
        self.last_rc_channels = None
        self.override_start_time = 0.0
        logger.info("RC override monitor reset")
    
    def get_status(self) -> Dict[str, Any]:
        """Get monitor status.
        
        Returns:
            Dictionary containing status information
        """
        return {
            'enabled': self.enabled,
            'active': self.state.active,
            'threshold': self.threshold,
            'neutral_timeout': self.neutral_timeout,
            'override_duration': self.get_override_duration(),
            'channel_changes': self.state.channel_changes,
            'last_activity': self.state.last_activity
        }
