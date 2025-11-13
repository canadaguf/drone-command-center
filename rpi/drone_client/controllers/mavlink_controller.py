"""
MAVLink controller for ArduCopter communication.
Handles flight control, telemetry, and RC override detection.
"""

import time
import logging
from typing import Dict, Any, Optional, Tuple
from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mavlink2

logger = logging.getLogger(__name__)

class MAVLinkController:
    """MAVLink controller for ArduCopter flight control."""
    
    def __init__(self, config: Dict[str, Any]):
        """Initialize MAVLink controller.
        
        Args:
            config: MAVLink configuration dictionary
        """
        self.connection_string = config.get('connection', '/dev/ttyAMA0')
        self.baud = config.get('baud', 256000)
        self.master = None
        self.connected = False
        
        # RC override detection
        self.rc_override_active = False
        self.last_rc_channels = None
        self.rc_override_threshold = 100
        
        # Telemetry cache
        self._telemetry_cache = {}
        self._last_telemetry_update = 0
        self._telemetry_update_interval = 0.5  # 2 Hz
        
    def connect(self) -> bool:
        """Connect to flight controller.
        
        Returns:
            True if connection successful, False otherwise
        """
        try:
            logger.info(f"Connecting to flight controller at {self.connection_string}...")
            self.master = mavutil.mavlink_connection(self.connection_string, baud=self.baud)
            
            # Wait for heartbeat
            logger.info("Waiting for heartbeat...")
            self.master.wait_heartbeat(timeout=10)
            logger.info("Heartbeat received from flight controller")
            
            self.connected = True
            return True
            
        except Exception as e:
            logger.error(f"Failed to connect to flight controller: {e}")
            self.connected = False
            return False
    
    def disconnect(self) -> None:
        """Disconnect from flight controller."""
        if self.master:
            self.master.close()
            self.master = None
        self.connected = False
        logger.info("Disconnected from flight controller")
    
    def arm(self) -> bool:
        """Arm the drone.
        
        Returns:
            True if command sent successfully, False otherwise
        """
        # Check if master connection exists (more reliable than self.connected flag)
        if not self.master:
            logger.error("Not connected to flight controller (master is None)")
            return False
        
        try:
            # Clear any active RC override before arming
            # ArduPilot blocks arming if RC override is active
            logger.info("Clearing RC override before arming...")
            self.clear_rc_override()
            
            logger.info("Arming drone...")
            # Use the same method as basic_arm.py which works
            self.master.arducopter_arm()
            logger.info("Arm command sent successfully")
            return True
        except AttributeError as e:
            logger.error(f"arducopter_arm method not available: {e}")
            return False
        except Exception as e:
            logger.error(f"Failed to arm drone: {e}")
            import traceback
            logger.debug(traceback.format_exc())
            return False
    
    def disarm(self) -> bool:
        """Disarm the drone.
        
        Returns:
            True if command sent successfully, False otherwise
        """
        # Check if master connection exists (more reliable than self.connected flag)
        if not self.master:
            logger.error("Not connected to flight controller (master is None)")
            return False
        
        try:
            # Clear any active RC override before disarming
            # This ensures clean disarm operation
            logger.info("Clearing RC override before disarming...")
            self.clear_rc_override()
            
            logger.info("Disarming drone...")
            # Use the same method as basic_arm.py which works
            self.master.arducopter_disarm()
            logger.info("Disarm command sent successfully")
            return True
        except AttributeError as e:
            logger.error(f"arducopter_disarm method not available: {e}")
            return False
        except Exception as e:
            logger.error(f"Failed to disarm drone: {e}")
            import traceback
            logger.debug(traceback.format_exc())
            return False
    
    def set_mode(self, mode_name: str) -> bool:
        """Set flight mode.
        
        Args:
            mode_name: Flight mode name (e.g., 'GUIDED', 'LAND', 'RTL', 'LOITER')
            
        Returns:
            True if command sent successfully, False otherwise
        """
        if not self.connected:
            logger.error("Not connected to flight controller")
            return False
        
        try:
            logger.info(f"Setting flight mode to {mode_name}")
            mode_id = self.master.mode_mapping()[mode_name]
            self.master.mav.set_mode_send(
                self.master.target_system,
                mavlink2.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id
            )
            return True
        except Exception as e:
            logger.error(f"Failed to set mode {mode_name}: {e}")
            return False
    
    def set_mode_loiter(self) -> bool:
        """Set flight mode to LOITER.
        
        LOITER mode maintains current position and altitude automatically.
        The pilot can control position with sticks, but when released, 
        the drone will hold position.
        
        Returns:
            True if command sent successfully, False otherwise
        """
        return self.set_mode('LOITER')
    
    def is_armed(self) -> bool:
        """Check if drone is armed.
        
        Returns:
            True if armed, False otherwise
        """
        if not self.master:
            return False
        
        try:
            # Force a fresh read of heartbeat to get current armed status
            # Read multiple messages to ensure we get the latest heartbeat
            is_armed = False
            for _ in range(5):  # Try up to 5 times
                msg = self.master.recv_match(type='HEARTBEAT', blocking=False, timeout=0.1)
                if msg:
                    is_armed = bool(msg.base_mode & mavlink2.MAV_MODE_FLAG_SAFETY_ARMED)
                    # Update cache with fresh data
                    self._telemetry_cache['armed'] = is_armed
                    break
            
            # If no heartbeat received, fall back to cached value
            if 'armed' not in self._telemetry_cache:
                # Try to get from telemetry cache
                telemetry = self.get_telemetry()
                is_armed = telemetry.get('armed', False)
            
            return is_armed
        except Exception as e:
            logger.error(f"Error checking armed status: {e}")
            # Fall back to cached value if available
            return self._telemetry_cache.get('armed', False)
    
    def clear_rc_override(self) -> bool:
        """Clear RC override by sending UINT16_MAX values.
        
        This releases RC override control back to the flight controller.
        ArduPilot requires this before arming can succeed.
        According to MAVLink spec, sending UINT16_MAX (65535) releases override.
        
        Returns:
            True if command sent successfully, False otherwise
        """
        if not self.master:
            logger.warning("Not connected to flight controller")
            return False
        
        try:
            # Send RC override with UINT16_MAX (65535) to clear override
            # This tells ArduPilot to release RC override and return control to RC receiver
            import time
            for _ in range(3):  # Send multiple times to ensure it's received
                self.master.mav.rc_channels_override_send(
                    self.master.target_system,
                    self.master.target_component,
                    65535, 65535, 65535, 65535,  # UINT16_MAX = no override
                    65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535
                )
                time.sleep(0.05)  # Small delay between sends
            
            logger.debug("RC override cleared (sent UINT16_MAX)")
            time.sleep(0.1)  # Small delay to let ArduPilot process
            return True
        except Exception as e:
            logger.warning(f"Failed to clear RC override: {e}")
            return False
    
    def send_rc_override(self, roll: int, pitch: int, yaw: int, throttle: int) -> bool:
        """Send RC override commands.
        
        Args:
            roll: Roll channel value (-100 to 100)
            pitch: Pitch channel value (-100 to 100)
            yaw: Yaw channel value (-100 to 100)
            throttle: Throttle channel value (0 to 100)
            
        Returns:
            True if command sent successfully, False otherwise
        """
        if not self.connected:
            logger.error("Not connected to flight controller")
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
                roll_rc, pitch_rc, throttle_rc, yaw_rc,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
            )
            return True
            
        except Exception as e:
            logger.error(f"Failed to send RC override: {e}")
            return False
    
    def request_takeoff(self, altitude: float) -> bool:
        """Request takeoff to specified altitude.
        
        Args:
            altitude: Target altitude in meters
            
        Returns:
            True if command sent successfully, False otherwise
        """
        if not self.connected:
            logger.error("Not connected to flight controller")
            return False
        
        try:
            logger.info(f"Requesting takeoff to {altitude}m")
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavlink2.MAV_CMD_NAV_TAKEOFF,
                0, 0, 0, 0, 0, 0, 0, altitude
            )
            return True
        except Exception as e:
            logger.error(f"Failed to request takeoff: {e}")
            return False
    
    def request_land(self) -> bool:
        """Request landing.
        
        Returns:
            True if command sent successfully, False otherwise
        """
        if not self.connected:
            logger.error("Not connected to flight controller")
            return False
        
        try:
            logger.info("Requesting landing")
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavlink2.MAV_CMD_NAV_LAND,
                0, 0, 0, 0, 0, 0, 0, 0
            )
            return True
        except Exception as e:
            logger.error(f"Failed to request landing: {e}")
            return False
    
    def get_telemetry(self) -> Dict[str, Any]:
        """Get current telemetry data.
        
        Returns:
            Dictionary containing telemetry data
        """
        if not self.connected:
            return {}
        
        # Check if we need to update telemetry
        current_time = time.time()
        if current_time - self._last_telemetry_update < self._telemetry_update_interval:
            return self._telemetry_cache
        
        try:
            # Read available messages
            while True:
                msg = self.master.recv_match(blocking=False, timeout=0.1)
                if msg is None:
                    break
                
                # Process different message types
                if msg.get_type() == 'GLOBAL_POSITION_INT':
                    self._telemetry_cache.update({
                        'lat': msg.lat / 1e7,
                        'lon': msg.lon / 1e7,
                        'altitude': msg.alt / 1000.0,
                        'relative_alt': msg.relative_alt / 1000.0
                    })
                
                elif msg.get_type() == 'SYS_STATUS':
                    self._telemetry_cache.update({
                        'battery_voltage': msg.voltage_battery / 1000.0,
                        'battery_remaining': msg.battery_remaining
                    })
                
                elif msg.get_type() == 'HEARTBEAT':
                    # Check if armed
                    is_armed = bool(msg.base_mode & mavlink2.MAV_MODE_FLAG_SAFETY_ARMED)
                    self._telemetry_cache.update({
                        'armed': is_armed,
                        'mode': self.master.flightmode,
                        'system_status': msg.system_status
                    })
                
                elif msg.get_type() == 'VFR_HUD':
                    self._telemetry_cache.update({
                        'groundspeed': msg.groundspeed,
                        'heading': msg.heading,
                        'throttle': msg.throttle
                    })
                
                elif msg.get_type() == 'RC_CHANNELS':
                    # Check for RC override
                    self._check_rc_override(msg)
                    self._telemetry_cache.update({
                        'rc_override_active': self.rc_override_active
                    })
            
            self._last_telemetry_update = current_time
            return self._telemetry_cache.copy()
            
        except Exception as e:
            logger.error(f"Error reading telemetry: {e}")
            return self._telemetry_cache.copy()
    
    def _check_rc_override(self, rc_msg) -> None:
        """Check if RC override is active.
        
        Args:
            rc_msg: RC_CHANNELS message
        """
        if self.last_rc_channels is None:
            self.last_rc_channels = [rc_msg.chan1_raw, rc_msg.chan2_raw, 
                                    rc_msg.chan3_raw, rc_msg.chan4_raw]
            return
        
        current_channels = [rc_msg.chan1_raw, rc_msg.chan2_raw, 
                           rc_msg.chan3_raw, rc_msg.chan4_raw]
        
        # Check if any channel changed significantly
        for i, (current, last) in enumerate(zip(current_channels, self.last_rc_channels)):
            if abs(current - last) > self.rc_override_threshold:
                if not self.rc_override_active:
                    logger.info(f"RC override detected on channel {i+1}")
                self.rc_override_active = True
                break
        else:
            # Check if all channels are near neutral
            neutral_range = 50  # ±50 from 1500
            all_neutral = all(abs(ch - 1500) < neutral_range for ch in current_channels)
            if all_neutral and self.rc_override_active:
                logger.info("RC override released - returning to autonomous")
                self.rc_override_active = False
        
        self.last_rc_channels = current_channels
    
    def is_rc_active(self) -> bool:
        """Check if RC override is currently active.
        
        Returns:
            True if RC override is active, False otherwise
        """
        return self.rc_override_active
    
    def is_connected(self) -> bool:
        """Check if connected to flight controller.
        
        Returns:
            True if connected, False otherwise
        """
        # Check both flag and actual connection object
        if not self.connected or not self.master:
            return False
        
        # Try to verify connection is still alive by checking if we can access it
        try:
            # Quick check - try to access target_system (won't fail if disconnected, but validates object)
            _ = self.master.target_system
            return True
        except:
            # Connection might be dead, reset flag
            self.connected = False
            return False
    
    def get_system_info(self) -> Dict[str, Any]:
        """Get system information.
        
        Returns:
            Dictionary containing system info
        """
        if not self.connected:
            return {}
        
        try:
            # Request autopilot version
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavlink2.MAV_CMD_REQUEST_MESSAGE,
                0, mavlink2.MAVLINK_MSG_ID_AUTOPILOT_VERSION,
                0, 0, 0, 0, 0, 0
            )
            
            # Wait for version message
            version_msg = self.master.recv_match(type='AUTOPILOT_VERSION', blocking=True, timeout=2)
            
            info = {
                'target_system': self.master.target_system,
                'target_component': self.master.target_component
            }
            
            if version_msg:
                info.update({
                    'firmware_version': version_msg.flight_sw_version,
                    'autopilot': version_msg.autopilot,
                    'capabilities': version_msg.capabilities
                })
            
            return info
            
        except Exception as e:
            logger.error(f"Error getting system info: {e}")
            return {}
