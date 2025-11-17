"""
Telemetry data collection and packaging.
Aggregates data from various sources for transmission to backend.
"""

import time
import logging
from typing import Dict, Any, Optional, List
from dataclasses import dataclass

logger = logging.getLogger(__name__)

@dataclass
class TelemetryData:
    """Telemetry data structure."""
    # GPS data
    lat: Optional[float] = None
    lon: Optional[float] = None
    altitude: Optional[float] = None
    relative_alt: Optional[float] = None
    
    # Flight data
    battery: Optional[float] = None
    battery_voltage: Optional[float] = None
    mode: Optional[str] = None
    armed: Optional[bool] = None
    velocity: Optional[float] = None
    heading: Optional[float] = None
    
    # TOF sensor data
    tof_forward: Optional[float] = None
    tof_down: Optional[float] = None
    
    # Tracking status
    tracking_status: str = "DISCONNECTED"
    rc_override_active: bool = False
    distance_mode: Optional[str] = None  # Current tracking distance mode
    
    # Timestamp
    timestamp: float = 0.0

class TelemetryCollector:
    """Telemetry data collector and aggregator."""
    
    def __init__(self):
        """Initialize telemetry collector."""
        self.last_telemetry = TelemetryData()
        self.update_interval = 0.5  # 2 Hz
        self.last_update = 0
        
        # Altitude reference for AGL (Above Ground Level) calculation
        self.initial_altitude_reference: Optional[float] = None
        self.altitude_reference_captured = False
        
    def update_from_mavlink(self, mavlink_data: Dict[str, Any]) -> None:
        """Update telemetry from MAVLink data.
        
        Args:
            mavlink_data: MAVLink telemetry data
        """
        if 'lat' in mavlink_data:
            self.last_telemetry.lat = mavlink_data['lat']
        if 'lon' in mavlink_data:
            self.last_telemetry.lon = mavlink_data['lon']
        if 'altitude' in mavlink_data:
            self.last_telemetry.altitude = mavlink_data['altitude']
        if 'relative_alt' in mavlink_data:
            self.last_telemetry.relative_alt = mavlink_data['relative_alt']
            
            # Auto-capture altitude reference if not set and we have valid relative_alt
            if not self.altitude_reference_captured and self.last_telemetry.relative_alt is not None:
                self.capture_altitude_reference()
        if 'battery_remaining' in mavlink_data:
            self.last_telemetry.battery = mavlink_data['battery_remaining']
        if 'battery_voltage' in mavlink_data:
            self.last_telemetry.battery_voltage = mavlink_data['battery_voltage']
        if 'mode' in mavlink_data:
            self.last_telemetry.mode = mavlink_data['mode']
        if 'armed' in mavlink_data:
            self.last_telemetry.armed = mavlink_data['armed']
        if 'groundspeed' in mavlink_data:
            self.last_telemetry.velocity = mavlink_data['groundspeed']
        if 'heading' in mavlink_data:
            self.last_telemetry.heading = mavlink_data['heading']
        if 'rc_override_active' in mavlink_data:
            self.last_telemetry.rc_override_active = mavlink_data['rc_override_active']
    
    def update_from_tof(self, tof_data: Dict[str, Dict[str, Any]]) -> None:
        """Update telemetry from TOF sensor data.
        
        Args:
            tof_data: TOF sensor readings
        """
        if 'forward' in tof_data and tof_data['forward'].get('valid', False):
            self.last_telemetry.tof_forward = tof_data['forward'].get('distance')
        if 'down' in tof_data and tof_data['down'].get('valid', False):
            self.last_telemetry.tof_down = tof_data['down'].get('distance')
    
    def update_tracking_status(self, status: str) -> None:
        """Update tracking status.
        
        Args:
            status: Tracking status string
        """
        self.last_telemetry.tracking_status = status
    
    def update_distance_mode(self, mode: str) -> None:
        """Update distance tracking mode.
        
        Args:
            mode: Distance mode ('close', 'medium', 'far')
        """
        self.last_telemetry.distance_mode = mode
    
    def get_telemetry(self) -> Dict[str, Any]:
        """Get current telemetry data.
        
        Returns:
            Dictionary containing telemetry data
        """
        current_time = time.time()
        
        # Update timestamp
        self.last_telemetry.timestamp = current_time
        
        # Check if we need to update
        if current_time - self.last_update < self.update_interval:
            return self._telemetry_to_dict()
        
        self.last_update = current_time
        
        # Package telemetry data
        telemetry = self._telemetry_to_dict()
        
        logger.debug(f"Telemetry updated: {telemetry}")
        return telemetry
    
    def _telemetry_to_dict(self) -> Dict[str, Any]:
        """Convert telemetry data to dictionary.
        
        Returns:
            Dictionary representation of telemetry
        """
        height_agl = self.get_height_agl()
        return {
            'gps': {
                'lat': self.last_telemetry.lat,
                'lon': self.last_telemetry.lon
            },
            'altitude': self.last_telemetry.altitude,
            'relative_alt': self.last_telemetry.relative_alt,
            'height_agl': height_agl,  # Height above ground level
            'altitude_reference': self.initial_altitude_reference,
            'battery': self.last_telemetry.battery,
            'battery_voltage': self.last_telemetry.battery_voltage,
            'mode': self.last_telemetry.mode,
            'armed': self.last_telemetry.armed,
            'velocity': self.last_telemetry.velocity,
            'heading': self.last_telemetry.heading,
            'tof_forward': self.last_telemetry.tof_forward,
            'tof_down': self.last_telemetry.tof_down,
            'tracking_status': self.last_telemetry.tracking_status,
            'rc_override_active': self.last_telemetry.rc_override_active,
            'distance_mode': self.last_telemetry.distance_mode,
            'timestamp': self.last_telemetry.timestamp
        }
    
    def get_gps_data(self) -> Dict[str, Any]:
        """Get GPS data only.
        
        Returns:
            Dictionary containing GPS data
        """
        return {
            'lat': self.last_telemetry.lat,
            'lon': self.last_telemetry.lon,
            'altitude': self.last_telemetry.altitude,
            'relative_alt': self.last_telemetry.relative_alt
        }
    
    def get_battery_data(self) -> Dict[str, Any]:
        """Get battery data only.
        
        Returns:
            Dictionary containing battery data
        """
        return {
            'battery': self.last_telemetry.battery,
            'battery_voltage': self.last_telemetry.battery_voltage
        }
    
    def get_flight_data(self) -> Dict[str, Any]:
        """Get flight data only.
        
        Returns:
            Dictionary containing flight data
        """
        return {
            'mode': self.last_telemetry.mode,
            'armed': self.last_telemetry.armed,
            'velocity': self.last_telemetry.velocity,
            'heading': self.last_telemetry.heading,
            'rc_override_active': self.last_telemetry.rc_override_active
        }
    
    def get_sensor_data(self) -> Dict[str, Any]:
        """Get sensor data only.
        
        Returns:
            Dictionary containing sensor data
        """
        return {
            'tof_forward': self.last_telemetry.tof_forward,
            'tof_down': self.last_telemetry.tof_down
        }
    
    def get_tracking_data(self) -> Dict[str, Any]:
        """Get tracking data only.
        
        Returns:
            Dictionary containing tracking data
        """
        return {
            'tracking_status': self.last_telemetry.tracking_status,
            'rc_override_active': self.last_telemetry.rc_override_active
        }
    
    def is_gps_valid(self) -> bool:
        """Check if GPS data is valid.
        
        Returns:
            True if GPS data is valid, False otherwise
        """
        return (self.last_telemetry.lat is not None and 
                self.last_telemetry.lon is not None and
                self.last_telemetry.altitude is not None)
    
    def is_battery_low(self, threshold: float = 20.0) -> bool:
        """Check if battery is low.
        
        Args:
            threshold: Battery percentage threshold
            
        Returns:
            True if battery is low, False otherwise
        """
        if self.last_telemetry.battery is None:
            return False
        return self.last_telemetry.battery < threshold
    
    def get_telemetry_age(self) -> float:
        """Get age of telemetry data.
        
        Returns:
            Age in seconds
        """
        return time.time() - self.last_telemetry.timestamp
    
    def capture_altitude_reference(self, force: bool = False) -> bool:
        """Capture initial altitude reference for AGL calculation.
        
        This should be called when the drone is on the ground (before takeoff).
        The reference altitude is used to calculate height above ground level (AGL).
        
        Args:
            force: If True, overwrite existing reference
            
        Returns:
            True if reference captured successfully, False otherwise
        """
        if self.altitude_reference_captured and not force:
            logger.debug("Altitude reference already captured")
            return False
        
        if self.last_telemetry.relative_alt is None:
            logger.warning("Cannot capture altitude reference - no relative_alt data available")
            return False
        
        self.initial_altitude_reference = self.last_telemetry.relative_alt
        self.altitude_reference_captured = True
        logger.info(f"Altitude reference captured: {self.initial_altitude_reference:.3f}m")
        return True
    
    def get_height_agl(self) -> Optional[float]:
        """Get height above ground level (AGL) in meters.
        
        Calculates: height_agl = current_relative_alt - initial_altitude_reference
        
        Returns:
            Height above ground level in meters, or None if reference not captured
        """
        if not self.altitude_reference_captured or self.initial_altitude_reference is None:
            return None
        
        if self.last_telemetry.relative_alt is None:
            return None
        
        height_agl = self.last_telemetry.relative_alt - self.initial_altitude_reference
        return max(0.0, height_agl)  # Ensure non-negative
    
    def reset_altitude_reference(self) -> None:
        """Reset altitude reference (called on landing/disarm)."""
        self.initial_altitude_reference = None
        self.altitude_reference_captured = False
        logger.info("Altitude reference reset")
    
    def reset(self) -> None:
        """Reset telemetry data."""
        self.last_telemetry = TelemetryData()
        self.last_update = 0
        self.reset_altitude_reference()
        logger.info("Telemetry data reset")
