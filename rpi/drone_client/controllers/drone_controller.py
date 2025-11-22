"""
Drone control using DroneKit.
High-level interface for drone operations.
"""

import logging
import time
import math
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
    
    def send_velocity_command(self, vx: float, vy: float, vz: float, yaw_rate: float = 0.0) -> bool:
        """Send velocity command in GUIDED mode.
        
        Args:
            vx: Forward velocity (m/s, positive = forward)
            vy: Right velocity (m/s, positive = right)
            vz: Up velocity (m/s, positive = up)
            yaw_rate: Yaw rate (rad/s, positive = clockwise)
            
        Returns:
            True if command sent successfully, False otherwise
        """
        if not self.is_connected():
            return False
        
        try:
            # Ensure GUIDED mode
            if self.vehicle.mode.name != 'GUIDED':
                if not self.set_mode('GUIDED'):
                    logger.error("Failed to set GUIDED mode for velocity command")
                    return False
            
            # Send velocity command using DroneKit
            # Note: DroneKit's velocity attribute sends velocity_ned (North, East, Down)
            # We need to convert from body frame (forward, right, up) to NED frame
            
            # Get current heading for conversion
            current_heading = math.radians(self.vehicle.heading)
            
            # Convert body frame to NED frame
            # Forward (vx) -> North component
            # Right (vy) -> East component
            # Up (vz) -> Down component (negative because NED uses Down as positive)
            velocity_north = vx * math.cos(current_heading) - vy * math.sin(current_heading)
            velocity_east = vx * math.sin(current_heading) + vy * math.cos(current_heading)
            velocity_down = -vz  # NED uses down as positive
            
            # Send velocity command
            self.vehicle.velocity = [velocity_north, velocity_east, velocity_down]
            
            # Send yaw rate if non-zero
            if abs(yaw_rate) > 0.01:
                # Convert yaw rate to heading change
                # For now, we'll use a simple approach: set heading relative to current
                # Note: This is a simplified approach - full implementation would use yaw rate directly
                # DroneKit doesn't directly support yaw rate, so we'll need to use MAVLink
                self._send_yaw_rate_mavlink(yaw_rate)
            
            return True
            
        except Exception as e:
            logger.error(f"Error sending velocity command: {e}")
            return False
    
    def _send_yaw_rate_mavlink(self, yaw_rate: float) -> None:
        """Send yaw rate command via MAVLink.
        
        Args:
            yaw_rate: Yaw rate in rad/s
        """
        try:
            # Use MAVLink message for yaw rate
            # MAV_CMD_DO_SET_ROI_LOCATION or use velocity_yaw_rate in velocity_ned
            # For ArduPilot, we can use SET_POSITION_TARGET_LOCAL_NED with velocity_yaw_rate
            msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
                0,  # time_boot_ms
                self.vehicle.target_system,
                self.vehicle.target_component,
                8,  # frame (MAV_FRAME_LOCAL_NED)
                0b0000111111000111,  # type_mask (ignore position, use velocity + yaw rate)
                0, 0, 0,  # x, y, z (ignored)
                0, 0, 0,  # vx, vy, vz (will be set separately)
                0, 0, 0,  # afx, afy, afz (ignored)
                0,  # yaw (ignored)
                yaw_rate  # yaw_rate
            )
            self.vehicle.send_mavlink(msg)
        except Exception as e:
            logger.debug(f"Could not send yaw rate via MAVLink: {e}")
    
    def send_position_command(self, dx: float, dy: float, dz: float) -> bool:
        """Send position command relative to current position.
        
        Args:
            dx: Forward offset (m, positive = forward)
            dy: Right offset (m, positive = right)
            dz: Up offset (m, positive = up)
            
        Returns:
            True if command sent successfully, False otherwise
        """
        if not self.is_connected():
            return False
        
        try:
            # Ensure GUIDED mode
            if self.vehicle.mode.name != 'GUIDED':
                if not self.set_mode('GUIDED'):
                    logger.error("Failed to set GUIDED mode for position command")
                    return False
            
            # Get current position
            current_location = self.vehicle.location.global_relative_frame
            
            # Calculate target position
            # Convert body frame offsets to NED frame
            current_heading = math.radians(self.vehicle.heading)
            
            # Forward (dx) -> North component
            # Right (dy) -> East component
            # Up (dz) -> Altitude increase
            offset_north = dx * math.cos(current_heading) - dy * math.sin(current_heading)
            offset_east = dx * math.sin(current_heading) + dy * math.cos(current_heading)
            
            # Create target location
            target_location = LocationGlobalRelative(
                current_location.lat,
                current_location.lon,
                current_location.alt + dz
            )
            
            # For horizontal movement, we need to calculate new lat/lon
            # Simplified: use simple offset (accurate for small distances)
            # 1 degree latitude ≈ 111km, 1 degree longitude ≈ 111km * cos(latitude)
            lat_offset = offset_north / 111000.0
            lon_offset = offset_east / (111000.0 * math.cos(math.radians(current_location.lat)))
            
            target_location = LocationGlobalRelative(
                current_location.lat + lat_offset,
                current_location.lon + lon_offset,
                current_location.alt + dz
            )
            
            # Send command
            self.vehicle.simple_goto(target_location)
            
            return True
            
        except Exception as e:
            logger.error(f"Error sending position command: {e}")
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

