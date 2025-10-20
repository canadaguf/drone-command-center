"""
Battery monitor for safety and failsafe management.
Monitors battery levels and triggers safety actions.
"""

import logging
import time
from typing import Dict, Any, Optional, Callable
from enum import Enum

logger = logging.getLogger(__name__)

class BatteryStatus(Enum):
    """Battery status levels."""
    NORMAL = "NORMAL"
    WARNING = "WARNING"
    CRITICAL = "CRITICAL"
    EMERGENCY = "EMERGENCY"

class BatteryMonitor:
    """Battery monitor for safety management."""
    
    def __init__(self, config: Dict[str, Any]):
        """Initialize battery monitor.
        
        Args:
            config: Battery configuration
        """
        self.warn_percent = config.get('warn_percent', 30)
        self.critical_percent = config.get('critical_percent', 20)
        self.auto_land_percent = config.get('auto_land_percent', 15)
        
        # State
        self.status = BatteryStatus.NORMAL
        self.last_battery_percent = None
        self.last_voltage = None
        self.last_update_time = 0.0
        
        # Status change tracking
        self.status_changes = []
        self.warning_sent = False
        self.critical_sent = False
        self.auto_land_triggered = False
        
        # Callbacks
        self.on_warning_callback = None
        self.on_critical_callback = None
        self.on_auto_land_callback = None
        
        logger.info(f"Battery monitor initialized: warn={self.warn_percent}%, critical={self.critical_percent}%, auto_land={self.auto_land_percent}%")
    
    def set_callbacks(self, on_warning: Optional[Callable] = None,
                     on_critical: Optional[Callable] = None,
                     on_auto_land: Optional[Callable] = None) -> None:
        """Set callback functions.
        
        Args:
            on_warning: Called when battery warning level reached
            on_critical: Called when battery critical level reached
            on_auto_land: Called when auto-land is triggered
        """
        self.on_warning_callback = on_warning
        self.on_critical_callback = on_critical
        self.on_auto_land_callback = on_auto_land
    
    def update(self, battery_percent: Optional[float], 
               battery_voltage: Optional[float] = None) -> BatteryStatus:
        """Update battery monitoring.
        
        Args:
            battery_percent: Battery percentage (0-100)
            battery_voltage: Battery voltage (optional)
            
        Returns:
            Current battery status
        """
        current_time = time.time()
        
        # Update state
        self.last_battery_percent = battery_percent
        self.last_voltage = battery_voltage
        self.last_update_time = current_time
        
        # Determine status
        new_status = self._determine_status(battery_percent)
        
        # Check for status changes
        if new_status != self.status:
            self._change_status(new_status)
        
        # Check for specific thresholds
        self._check_thresholds(battery_percent)
        
        return self.status
    
    def _determine_status(self, battery_percent: Optional[float]) -> BatteryStatus:
        """Determine battery status from percentage.
        
        Args:
            battery_percent: Battery percentage
            
        Returns:
            Battery status
        """
        if battery_percent is None:
            return BatteryStatus.NORMAL
        
        if battery_percent <= self.auto_land_percent:
            return BatteryStatus.EMERGENCY
        elif battery_percent <= self.critical_percent:
            return BatteryStatus.CRITICAL
        elif battery_percent <= self.warn_percent:
            return BatteryStatus.WARNING
        else:
            return BatteryStatus.NORMAL
    
    def _change_status(self, new_status: BatteryStatus) -> None:
        """Change battery status.
        
        Args:
            new_status: New battery status
        """
        old_status = self.status
        self.status = new_status
        
        # Record status change
        self.status_changes.append({
            'timestamp': time.time(),
            'old_status': old_status.value,
            'new_status': new_status.value,
            'battery_percent': self.last_battery_percent,
            'voltage': self.last_voltage
        })
        
        # Keep only last 10 changes
        if len(self.status_changes) > 10:
            self.status_changes = self.status_changes[-10:]
        
        logger.info(f"Battery status changed: {old_status.value} → {new_status.value} ({self.last_battery_percent}%)")
    
    def _check_thresholds(self, battery_percent: Optional[float]) -> None:
        """Check for threshold crossings.
        
        Args:
            battery_percent: Battery percentage
        """
        if battery_percent is None:
            return
        
        # Warning threshold
        if battery_percent <= self.warn_percent and not self.warning_sent:
            self.warning_sent = True
            if self.on_warning_callback:
                self.on_warning_callback(battery_percent)
            logger.warning(f"Battery warning: {battery_percent}%")
        
        # Critical threshold
        if battery_percent <= self.critical_percent and not self.critical_sent:
            self.critical_sent = True
            if self.on_critical_callback:
                self.on_critical_callback(battery_percent)
            logger.error(f"Battery critical: {battery_percent}%")
        
        # Auto-land threshold
        if battery_percent <= self.auto_land_percent and not self.auto_land_triggered:
            self.auto_land_triggered = True
            if self.on_auto_land_callback:
                self.on_auto_land_callback(battery_percent)
            logger.critical(f"Battery emergency - auto-land triggered: {battery_percent}%")
    
    def get_status(self) -> BatteryStatus:
        """Get current battery status.
        
        Returns:
            Current battery status
        """
        return self.status
    
    def get_status_string(self) -> str:
        """Get current status as string.
        
        Returns:
            Status string
        """
        return self.status.value
    
    def is_battery_low(self) -> bool:
        """Check if battery is low.
        
        Returns:
            True if battery is low, False otherwise
        """
        return self.status in [BatteryStatus.WARNING, BatteryStatus.CRITICAL, BatteryStatus.EMERGENCY]
    
    def is_auto_land_triggered(self) -> bool:
        """Check if auto-land has been triggered.
        
        Returns:
            True if auto-land triggered, False otherwise
        """
        return self.auto_land_triggered
    
    def get_battery_info(self) -> Dict[str, Any]:
        """Get current battery information.
        
        Returns:
            Dictionary containing battery info
        """
        return {
            'percent': self.last_battery_percent,
            'voltage': self.last_voltage,
            'status': self.status.value,
            'last_update': self.last_update_time,
            'is_low': self.is_battery_low(),
            'auto_land_triggered': self.auto_land_triggered
        }
    
    def get_status_changes(self) -> list:
        """Get recent status changes.
        
        Returns:
            List of recent status changes
        """
        return self.status_changes.copy()
    
    def get_config(self) -> Dict[str, Any]:
        """Get monitor configuration.
        
        Returns:
            Dictionary containing configuration
        """
        return {
            'warn_percent': self.warn_percent,
            'critical_percent': self.critical_percent,
            'auto_land_percent': self.auto_land_percent
        }
    
    def set_thresholds(self, warn_percent: float, critical_percent: float, 
                      auto_land_percent: float) -> None:
        """Set battery thresholds.
        
        Args:
            warn_percent: Warning threshold percentage
            critical_percent: Critical threshold percentage
            auto_land_percent: Auto-land threshold percentage
        """
        self.warn_percent = warn_percent
        self.critical_percent = critical_percent
        self.auto_land_percent = auto_land_percent
        
        logger.info(f"Battery thresholds updated: warn={warn_percent}%, critical={critical_percent}%, auto_land={auto_land_percent}%")
    
    def reset(self) -> None:
        """Reset monitor state."""
        self.status = BatteryStatus.NORMAL
        self.last_battery_percent = None
        self.last_voltage = None
        self.last_update_time = 0.0
        self.status_changes.clear()
        self.warning_sent = False
        self.critical_sent = False
        self.auto_land_triggered = False
        
        logger.info("Battery monitor reset")
    
    def get_estimated_flight_time(self, current_consumption: float = 0.0) -> Optional[float]:
        """Estimate remaining flight time.
        
        Args:
            current_consumption: Current power consumption in watts
            
        Returns:
            Estimated flight time in minutes, None if cannot estimate
        """
        if self.last_battery_percent is None or current_consumption <= 0:
            return None
        
        # Simple estimation based on percentage and consumption
        # This is a rough estimate - real implementation would need
        # battery capacity and consumption history
        remaining_percent = self.last_battery_percent
        estimated_time = (remaining_percent / 100.0) * (60.0 / current_consumption)
        
        return max(0.0, estimated_time)
    
    def get_voltage_status(self) -> str:
        """Get voltage status string.
        
        Returns:
            Voltage status string
        """
        if self.last_voltage is None:
            return "Unknown"
        
        if self.last_voltage >= 22.0:
            return "Good"
        elif self.last_voltage >= 21.0:
            return "Low"
        elif self.last_voltage >= 20.0:
            return "Critical"
        else:
            return "Emergency"
