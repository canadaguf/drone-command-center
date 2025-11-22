"""
Drone control using DroneKit.
High-level interface for drone operations.
"""

import logging
import time
from typing import Dict, Any, Optional
from dronekit import connect, VehicleMode, LocationGlobalRelative
import dronekit

logger = logging.getLogger(__name__)


class DroneController:
    """Drone control using DroneKit."""
    
    def __init__(self, config: Dict[str, Any]):
        """Initialize drone controller.
        
        Args:
            config: MAVLink configuration dictionary
        """
        self.connection_string = config.get('connection', '/dev/ttyAMA0')
        self.baud = config.get('baud', 57600)
        self.takeoff_altitude = config.get('takeoff_altitude', 1.5)
        
        self.vehicle = None
        self.connected = False
        
    def connect(self) -> bool:
        """Connect to the vehicle.
        
        Returns:
            True if connection successful, False otherwise
        """
        try:
            # Build connection string with baud rate
            if self.connection_string.startswith('/dev/'):
                # Serial connection
                conn_str = f"{self.connection_string}?baud={self.baud}"
            else:
                conn_str = self.connection_string
            
            logger.info(f"Connecting to vehicle at {conn_str}...")
            self.vehicle = connect(conn_str, wait_ready=['autopilot_version'])
            self.connected = True
            
            logger.info(f"Connected to vehicle: {self.vehicle.version}")
            logger.info(f"Autopilot: {self.vehicle.version.autopilot_type}")
            logger.info(f"Firmware: {self.vehicle.version.major}.{self.vehicle.version.minor}")
            
            return True
            
        except Exception as e:
            logger.error(f"Failed to connect to vehicle: {e}")
            self.connected = False
            return False
    
    def disconnect(self) -> None:
        """Disconnect from the vehicle."""
        if self.vehicle:
            try:
                self.vehicle.close()
                logger.info("Disconnected from vehicle")
            except Exception as e:
                logger.error(f"Error disconnecting: {e}")
            finally:
                self.vehicle = None
                self.connected = False
    
    def is_connected(self) -> bool:
        """Check if connected to vehicle.
        
        Returns:
            True if connected, False otherwise
        """
        return self.connected and self.vehicle is not None
    
    def is_armed(self) -> bool:
        """Check if vehicle is armed.
        
        Returns:
            True if armed, False otherwise
        """
        if not self.is_connected():
            return False
        return self.vehicle.armed
    
    def get_mode(self) -> Optional[str]:
        """Get current flight mode.
        
        Returns:
            Current mode name or None if not connected
        """
        if not self.is_connected():
            return None
        return self.vehicle.mode.name
    
    def arm(self) -> bool:
        """Arm the vehicle.
        
        Returns:
            True if arming successful, False otherwise
        """
        if not self.is_connected():
            logger.error("Cannot arm: not connected")
            return False
        
        try:
            # Check if already armed
            if self.vehicle.armed:
                logger.info("Vehicle already armed")
                return True
            
            # Set mode to GUIDED (required for arming in some cases)
            if self.vehicle.mode.name != 'GUIDED':
                logger.info("Setting mode to GUIDED for arming")
                self.vehicle.mode = VehicleMode("GUIDED")
                while self.vehicle.mode.name != 'GUIDED':
                    time.sleep(0.1)
            
            # Arm the vehicle
            logger.info("Arming vehicle...")
            self.vehicle.armed = True
            
            # Wait for arming to complete
            timeout = 10  # seconds
            start_time = time.time()
            while not self.vehicle.armed and (time.time() - start_time) < timeout:
                time.sleep(0.1)
            
            if self.vehicle.armed:
                logger.info("Vehicle armed successfully")
                return True
            else:
                logger.error("Arming timeout - vehicle not armed")
                return False
                
        except Exception as e:
            logger.error(f"Error arming vehicle: {e}")
            return False
    
    def disarm(self) -> bool:
        """Disarm the vehicle.
        
        Returns:
            True if disarming successful, False otherwise
        """
        if not self.is_connected():
            logger.error("Cannot disarm: not connected")
            return False
        
        try:
            # Check if already disarmed
            if not self.vehicle.armed:
                logger.info("Vehicle already disarmed")
                return True
            
            # Disarm the vehicle
            logger.info("Disarming vehicle...")
            self.vehicle.armed = False
            
            # Wait for disarming to complete
            timeout = 5  # seconds
            start_time = time.time()
            while self.vehicle.armed and (time.time() - start_time) < timeout:
                time.sleep(0.1)
            
            if not self.vehicle.armed:
                logger.info("Vehicle disarmed successfully")
                return True
            else:
                logger.error("Disarming timeout - vehicle still armed")
                return False
                
        except Exception as e:
            logger.error(f"Error disarming vehicle: {e}")
            return False
    
    def set_mode(self, mode_name: str) -> bool:
        """Set flight mode.
        
        Args:
            mode_name: Name of the flight mode (e.g., 'GUIDED', 'LOITER', 'LAND')
            
        Returns:
            True if mode change successful, False otherwise
        """
        if not self.is_connected():
            logger.error(f"Cannot set mode {mode_name}: not connected")
            return False
        
        try:
            logger.info(f"Setting mode to {mode_name}...")
            self.vehicle.mode = VehicleMode(mode_name)
            
            # Wait for mode change to complete
            timeout = 5  # seconds
            start_time = time.time()
            while self.vehicle.mode.name != mode_name and (time.time() - start_time) < timeout:
                time.sleep(0.1)
            
            if self.vehicle.mode.name == mode_name:
                logger.info(f"Mode set to {mode_name} successfully")
                return True
            else:
                logger.error(f"Mode change timeout - current mode: {self.vehicle.mode.name}")
                return False
                
        except Exception as e:
            logger.error(f"Error setting mode {mode_name}: {e}")
            return False
    
    def set_mode_loiter(self) -> bool:
        """Set LOITER mode (position hold).
        
        Returns:
            True if successful, False otherwise
        """
        return self.set_mode('LOITER')
    
    def simple_takeoff(self, altitude: Optional[float] = None) -> bool:
        """Take off to specified altitude.
        
        Args:
            altitude: Target altitude in meters (AGL). If None, uses config default.
            
        Returns:
            True if takeoff command sent successfully, False otherwise
        """
        if not self.is_connected():
            logger.error("Cannot takeoff: not connected")
            return False
        
        if not self.vehicle.armed:
            logger.error("Cannot takeoff: vehicle not armed")
            return False
        
        try:
            # Use provided altitude or default
            target_altitude = altitude if altitude is not None else self.takeoff_altitude
            
            # Ensure GUIDED mode
            if self.vehicle.mode.name != 'GUIDED':
                if not self.set_mode('GUIDED'):
                    logger.error("Failed to set GUIDED mode for takeoff")
                    return False
            
            logger.info(f"Taking off to {target_altitude}m...")
            
            # Use DroneKit's simple_takeoff
            self.vehicle.simple_takeoff(target_altitude)
            
            logger.info("Takeoff command sent successfully")
            return True
            
        except Exception as e:
            logger.error(f"Error during takeoff: {e}")
            return False
    
    def simple_land(self) -> bool:
        """Land the vehicle.
        
        Returns:
            True if land command sent successfully, False otherwise
        """
        if not self.is_connected():
            logger.error("Cannot land: not connected")
            return False
        
        try:
            # Set LAND mode
            logger.info("Initiating landing...")
            if not self.set_mode('LAND'):
                logger.error("Failed to set LAND mode")
                return False
            
            logger.info("Land command sent successfully")
            return True
            
        except Exception as e:
            logger.error(f"Error during landing: {e}")
            return False
    
    def send_rc_override(self, roll: int, pitch: int, yaw: int, throttle: int) -> bool:
        """Send RC override commands.
        
        Args:
            roll: Roll channel value (1100-1900, 1500 = neutral)
            pitch: Pitch channel value (1100-1900, 1500 = neutral)
            yaw: Yaw channel value (1100-1900, 1500 = neutral)
            throttle: Throttle channel value (1100-1900, 1500 = neutral)
            
        Returns:
            True if command sent successfully, False otherwise
        """
        if not self.is_connected():
            return False
        
        try:
            # Clamp values to valid range
            roll = max(1100, min(1900, roll))
            pitch = max(1100, min(1900, pitch))
            yaw = max(1100, min(1900, yaw))
            throttle = max(1100, min(1900, throttle))
            
            # Send RC override
            self.vehicle.channels.overrides = {
                '1': roll,
                '2': pitch,
                '3': throttle,
                '4': yaw
            }
            
            return True
            
        except Exception as e:
            logger.error(f"Error sending RC override: {e}")
            return False
    
    def clear_rc_override(self) -> bool:
        """Clear RC override (return control to flight controller).
        
        Returns:
            True if successful, False otherwise
        """
        if not self.is_connected():
            return False
        
        try:
            self.vehicle.channels.overrides = {}
            logger.info("RC override cleared")
            return True
            
        except Exception as e:
            logger.error(f"Error clearing RC override: {e}")
            return False
    
    def get_telemetry(self) -> Dict[str, Any]:
        """Get basic telemetry from DroneKit vehicle.
        
        Note: This is a minimal interface. For detailed telemetry,
        use TelemetryReader with pymavlink.
        
        Returns:
            Dictionary with basic telemetry data
        """
        if not self.is_connected():
            return {}
        
        try:
            return {
                'armed': self.vehicle.armed,
                'mode': self.vehicle.mode.name,
                'location': {
                    'lat': self.vehicle.location.global_frame.lat,
                    'lon': self.vehicle.location.global_frame.lon,
                    'alt': self.vehicle.location.global_frame.alt
                },
                'relative_alt': self.vehicle.location.global_relative_frame.alt,
                'heading': self.vehicle.heading,
                'groundspeed': self.vehicle.groundspeed,
                'airspeed': self.vehicle.airspeed,
                'battery': {
                    'voltage': self.vehicle.battery.voltage,
                    'current': self.vehicle.battery.current,
                    'level': self.vehicle.battery.level
                }
            }
        except Exception as e:
            logger.error(f"Error getting telemetry: {e}")
            return {}

