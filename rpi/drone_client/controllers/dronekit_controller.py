"""
DroneKit-based controller for simplified drone operations.
Uses DroneKit for basic flight operations (arm, takeoff, land, guided movement).
Falls back to pymavlink for advanced features like RC override.
"""

# Python 3.13 compatibility fix for DroneKit
# collections.MutableMapping was removed in Python 3.13
import collections
import collections.abc
if not hasattr(collections, 'MutableMapping'):
    collections.MutableMapping = collections.abc.MutableMapping

import logging
import time
from typing import Dict, Any, Optional
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mavlink2

logger = logging.getLogger(__name__)


class DroneKitController:
    """DroneKit-based controller for simplified drone operations."""
    
    def __init__(self, config: Dict[str, Any]):
        """Initialize DroneKit controller.
        
        Args:
            config: Controller configuration dictionary
        """
        self.connection_string = config.get('connection', '/dev/ttyAMA0')
        self.baud = config.get('baud', 256000)
        self.vehicle = None
        self.master = None  # Keep pymavlink connection for RC override
        self.connected = False
        
        # Telemetry cache
        self._telemetry_cache = {}
        self._last_telemetry_update = 0
        self._telemetry_update_interval = 0.5  # 2 Hz
        
    def connect(self) -> bool:
        """Connect to flight controller using DroneKit.
        
        Returns:
            True if connection successful, False otherwise
        """
        try:
            logger.info(f"Connecting to flight controller at {self.connection_string}...")
            
            # Connect using DroneKit
            connection_str = f"{self.connection_string}?baud={self.baud}"
            self.vehicle = connect(connection_str, wait_ready=False, timeout=10)
            
            # Wait for vehicle to be ready (but don't wait for all attributes)
            logger.info("Waiting for vehicle to initialize...")
            self.vehicle.wait_ready(['autopilot_version'], timeout=10)
            
            logger.info(f"Connected! Vehicle type: {self.vehicle.vehicle_type}")
            logger.info(f"Autopilot version: {self.vehicle.version}")
            
            # Also create pymavlink connection for RC override
            try:
                self.master = mavutil.mavlink_connection(self.connection_string, baud=self.baud)
                self.master.wait_heartbeat(timeout=5)
                logger.info("pymavlink connection established for RC override")
            except Exception as e:
                logger.warning(f"Failed to create pymavlink connection: {e}")
                logger.warning("RC override features will be unavailable")
            
            self.connected = True
            return True
            
        except Exception as e:
            logger.error(f"Failed to connect to flight controller: {e}")
            self.connected = False
            return False
    
    def disconnect(self) -> None:
        """Disconnect from flight controller."""
        if self.vehicle:
            try:
                self.vehicle.close()
            except:
                pass
            self.vehicle = None
        
        if self.master:
            try:
                self.master.close()
            except:
                pass
            self.master = None
        
        self.connected = False
        logger.info("Disconnected from flight controller")
    
    def arm(self) -> bool:
        """Arm the drone using DroneKit.
        
        Returns:
            True if armed successfully, False otherwise
        """
        if not self.connected or not self.vehicle:
            logger.error("Not connected to flight controller")
            return False
        
        try:
            # Clear RC override before arming
            if self.master:
                self.clear_rc_override()
                time.sleep(0.2)
            
            # Check if already armed
            if self.vehicle.armed:
                logger.info("Drone already armed")
                return True
            
            # Arm the vehicle
            logger.info("Arming drone...")
            self.vehicle.armed = True
            
            # Wait for arming to complete (with timeout)
            timeout = 10
            start_time = time.time()
            while not self.vehicle.armed and (time.time() - start_time) < timeout:
                time.sleep(0.1)
            
            if self.vehicle.armed:
                logger.info("✓ Drone armed successfully")
                return True
            else:
                logger.warning("⚠️ Arm command sent but drone not armed - check pre-arm checks")
                return False
                
        except Exception as e:
            logger.error(f"Failed to arm drone: {e}")
            return False
    
    def disarm(self) -> bool:
        """Disarm the drone using DroneKit.
        
        Returns:
            True if disarmed successfully, False otherwise
        """
        if not self.connected or not self.vehicle:
            logger.error("Not connected to flight controller")
            return False
        
        try:
            # Clear RC override before disarming
            if self.master:
                self.clear_rc_override()
            
            # Check if already disarmed
            if not self.vehicle.armed:
                logger.info("Drone already disarmed")
                return True
            
            # Disarm the vehicle
            logger.info("Disarming drone...")
            self.vehicle.armed = False
            
            # Wait for disarming to complete (with timeout)
            timeout = 5
            start_time = time.time()
            while self.vehicle.armed and (time.time() - start_time) < timeout:
                time.sleep(0.1)
            
            if not self.vehicle.armed:
                logger.info("✓ Drone disarmed successfully")
                return True
            else:
                logger.warning("⚠️ Disarm command sent but drone still armed")
                return False
                
        except Exception as e:
            logger.error(f"Failed to disarm drone: {e}")
            return False
    
    def is_armed(self) -> bool:
        """Check if drone is armed.
        
        Returns:
            True if armed, False otherwise
        """
        if not self.vehicle:
            return False
        return self.vehicle.armed
    
    def set_mode(self, mode_name: str) -> bool:
        """Set flight mode using DroneKit.
        
        Args:
            mode_name: Flight mode name (e.g., 'GUIDED', 'LAND', 'RTL', 'LOITER')
            
        Returns:
            True if mode set successfully, False otherwise
        """
        if not self.connected or not self.vehicle:
            logger.error("Not connected to flight controller")
            return False
        
        try:
            logger.info(f"Setting flight mode to {mode_name}")
            self.vehicle.mode = VehicleMode(mode_name)
            
            # Wait for mode to change (with timeout)
            timeout = 5
            start_time = time.time()
            while self.vehicle.mode.name != mode_name and (time.time() - start_time) < timeout:
                time.sleep(0.1)
            
            if self.vehicle.mode.name == mode_name:
                logger.info(f"✓ Flight mode set to {mode_name}")
                return True
            else:
                logger.warning(f"⚠️ Mode change requested but current mode is {self.vehicle.mode.name}")
                return False
                
        except Exception as e:
            logger.error(f"Failed to set mode {mode_name}: {e}")
            return False
    
    def set_mode_loiter(self) -> bool:
        """Set flight mode to LOITER.
        
        Returns:
            True if mode set successfully, False otherwise
        """
        return self.set_mode('LOITER')
    
    def simple_takeoff(self, altitude: float) -> bool:
        """Simple takeoff to specified altitude using DroneKit.
        
        Args:
            altitude: Target altitude in meters above ground
            
        Returns:
            True if takeoff command sent successfully, False otherwise
        """
        if not self.connected or not self.vehicle:
            logger.error("Not connected to flight controller")
            return False
        
        try:
            # Ensure vehicle is in GUIDED mode
            if self.vehicle.mode.name != 'GUIDED':
                logger.info("Switching to GUIDED mode for takeoff...")
                self.set_mode('GUIDED')
                time.sleep(1)
            
            # Ensure vehicle is armed
            if not self.vehicle.armed:
                logger.warning("Vehicle not armed - attempting to arm...")
                if not self.arm():
                    logger.error("Failed to arm vehicle for takeoff")
                    return False
            
            logger.info(f"Taking off to {altitude}m...")
            self.vehicle.simple_takeoff(altitude)
            
            logger.info("✓ Takeoff command sent")
            return True
            
        except Exception as e:
            logger.error(f"Failed to initiate takeoff: {e}")
            return False
    
    def simple_land(self) -> bool:
        """Simple landing using DroneKit LAND mode.
        
        Returns:
            True if landing command sent successfully, False otherwise
        """
        return self.set_mode('LAND')
    
    def simple_goto(self, lat: float, lon: float, altitude: float) -> bool:
        """Move to a specific location using DroneKit simple_goto.
        
        Args:
            lat: Target latitude
            lon: Target longitude
            altitude: Target altitude in meters above ground
            
        Returns:
            True if command sent successfully, False otherwise
        """
        if not self.connected or not self.vehicle:
            logger.error("Not connected to flight controller")
            return False
        
        try:
            # Ensure vehicle is in GUIDED mode
            if self.vehicle.mode.name != 'GUIDED':
                logger.info("Switching to GUIDED mode for movement...")
                self.set_mode('GUIDED')
                time.sleep(1)
            
            logger.info(f"Moving to location: lat={lat}, lon={lon}, alt={altitude}m")
            target = LocationGlobalRelative(lat, lon, altitude)
            self.vehicle.simple_goto(target)
            
            logger.info("✓ Movement command sent")
            return True
            
        except Exception as e:
            logger.error(f"Failed to send goto command: {e}")
            return False
    
    def clear_rc_override(self) -> bool:
        """Clear RC override using pymavlink.
        
        Returns:
            True if command sent successfully, False otherwise
        """
        if not self.master:
            return False
        
        try:
            # Send RC override with UINT16_MAX (65535) to clear override
            for _ in range(3):
                self.master.mav.rc_channels_override_send(
                    self.master.target_system,
                    self.master.target_component,
                    65535, 65535, 65535, 65535,  # Channels 1-4: UINT16_MAX = no override
                    65535, 65535, 65535, 65535   # Channels 5-8: UINT16_MAX = no override
                )
                time.sleep(0.05)
            
            logger.debug("RC override cleared")
            return True
        except Exception as e:
            logger.warning(f"Failed to clear RC override: {e}")
            return False
    
    def send_rc_override(self, roll: int, pitch: int, yaw: int, throttle: int) -> bool:
        """Send RC override commands using pymavlink.
        
        Args:
            roll: Roll channel value (-100 to 100)
            pitch: Pitch channel value (-100 to 100)
            yaw: Yaw channel value (-100 to 100)
            throttle: Throttle channel value (0 to 100)
            
        Returns:
            True if command sent successfully, False otherwise
        """
        if not self.master:
            logger.error("pymavlink connection not available for RC override")
            return False
        
        try:
            # Clamp values to valid ranges
            roll = max(-100, min(100, roll))
            pitch = max(-100, min(100, pitch))
            yaw = max(-100, min(100, yaw))
            throttle = max(0, min(100, throttle))
            
            # Convert to RC channel values (1000-2000 range)
            roll_rc = int(1500 + roll * 5)
            pitch_rc = int(1500 + pitch * 5)
            yaw_rc = int(1500 + yaw * 5)
            throttle_rc = int(1000 + throttle * 10)
            
            self.master.mav.rc_channels_override_send(
                self.master.target_system,
                self.master.target_component,
                roll_rc, pitch_rc, throttle_rc, yaw_rc,  # Channels 1-4
                0, 0, 0, 0  # Channels 5-8 (unused)
            )
            return True
            
        except Exception as e:
            logger.error(f"Failed to send RC override: {e}")
            return False
    
    def get_telemetry(self) -> Dict[str, Any]:
        """Get current telemetry data from DroneKit vehicle.
        
        Returns:
            Dictionary containing telemetry data
        """
        if not self.connected or not self.vehicle:
            return {}
        
        # Check if we need to update telemetry
        current_time = time.time()
        if current_time - self._last_telemetry_update < self._telemetry_update_interval:
            return self._telemetry_cache
        
        try:
            # Get telemetry from DroneKit vehicle
            self._telemetry_cache = {
                'armed': self.vehicle.armed,
                'mode': self.vehicle.mode.name,
                'system_status': self.vehicle.system_status.state,
                
                # Location
                'lat': self.vehicle.location.global_frame.lat if self.vehicle.location.global_frame else None,
                'lon': self.vehicle.location.global_frame.lon if self.vehicle.location.global_frame else None,
                'altitude': self.vehicle.location.global_frame.alt if self.vehicle.location.global_frame else None,
                'relative_alt': self.vehicle.location.global_relative_frame.alt if self.vehicle.location.global_relative_frame else None,
                
                # Attitude
                'roll': self.vehicle.attitude.roll if self.vehicle.attitude else None,
                'pitch': self.vehicle.attitude.pitch if self.vehicle.attitude else None,
                'yaw': self.vehicle.attitude.yaw if self.vehicle.attitude else None,
                
                # Velocity
                'groundspeed': self.vehicle.groundspeed if hasattr(self.vehicle, 'groundspeed') else None,
                'airspeed': self.vehicle.airspeed if hasattr(self.vehicle, 'airspeed') else None,
                
                # Battery
                'battery_voltage': self.vehicle.battery.voltage if self.vehicle.battery else None,
                'battery_remaining': self.vehicle.battery.level if self.vehicle.battery else None,
                
                # GPS
                'gps_satellites': self.vehicle.gps_0.satellites_visible if self.vehicle.gps_0 else None,
                'gps_fix_type': self.vehicle.gps_0.fix_type if self.vehicle.gps_0 else None,
                
                # Heading
                'heading': self.vehicle.heading if hasattr(self.vehicle, 'heading') else None,
            }
            
            self._last_telemetry_update = current_time
            return self._telemetry_cache.copy()
            
        except Exception as e:
            logger.error(f"Error reading telemetry: {e}")
            return self._telemetry_cache.copy()
    
    def is_connected(self) -> bool:
        """Check if connected to flight controller.
        
        Returns:
            True if connected, False otherwise
        """
        if not self.connected or not self.vehicle:
            return False
        
        try:
            # Check if vehicle is still responsive
            _ = self.vehicle.armed
            return True
        except:
            self.connected = False
            return False
    
    def get_system_info(self) -> Dict[str, Any]:
        """Get system information.
        
        Returns:
            Dictionary containing system info
        """
        if not self.connected or not self.vehicle:
            return {}
        
        try:
            info = {
                'vehicle_type': self.vehicle.vehicle_type,
                'autopilot_version': self.vehicle.version,
                'firmware_version': self.vehicle.version.flight_sw_version if self.vehicle.version else None,
            }
            
            if self.master:
                info.update({
                    'target_system': self.master.target_system,
                    'target_component': self.master.target_component
                })
            
            return info
            
        except Exception as e:
            logger.error(f"Error getting system info: {e}")
            return {}

