"""
Telemetry collection using pymavlink.
Direct MAVLink connection for real-time telemetry data.
"""

import logging
import time
from typing import Dict, Any, Optional
from pymavlink import mavutil

logger = logging.getLogger(__name__)


class TelemetryReader:
    """Telemetry reader using pymavlink."""
    
    def __init__(self, config: Dict[str, Any]):
        """Initialize telemetry reader.
        
        Args:
            config: MAVLink configuration dictionary
        """
        self.connection_string = config.get('connection', '/dev/ttyAMA0')
        self.baud = config.get('baud', 57600)
        
        self.master = None
        self.connected = False
        
        # Telemetry data cache
        self.last_telemetry = {}
        self.last_update = 0
        
    def connect(self) -> bool:
        """Connect to the vehicle via MAVLink.
        
        Returns:
            True if connection successful, False otherwise
        """
        try:
            # Build connection string
            if self.connection_string.startswith('/dev/'):
                # Serial connection
                conn_str = f"{self.connection_string}:{self.baud}"
            else:
                conn_str = self.connection_string
            
            logger.info(f"Connecting to MAVLink at {conn_str}...")
            self.master = mavutil.mavlink_connection(conn_str)
            
            # Wait for heartbeat to confirm connection
            logger.info("Waiting for heartbeat...")
            self.master.wait_heartbeat(timeout=10)
            
            self.connected = True
            logger.info(f"Connected to MAVLink (system {self.master.target_system}, component {self.master.target_component})")
            
            return True
            
        except Exception as e:
            logger.error(f"Failed to connect to MAVLink: {e}")
            self.connected = False
            return False
    
    def disconnect(self) -> None:
        """Disconnect from MAVLink."""
        if self.master:
            try:
                self.master.close()
                logger.info("Disconnected from MAVLink")
            except Exception as e:
                logger.error(f"Error disconnecting: {e}")
            finally:
                self.master = None
                self.connected = False
    
    def is_connected(self) -> bool:
        """Check if connected.
        
        Returns:
            True if connected, False otherwise
        """
        return self.connected and self.master is not None
    
    def read_telemetry(self) -> Dict[str, Any]:
        """Read telemetry data from MAVLink messages.
        
        This method should be called periodically (e.g., every 0.5s for 2Hz).
        It processes available MAVLink messages and updates telemetry cache.
        
        Returns:
            Dictionary with telemetry data
        """
        if not self.is_connected():
            return self.last_telemetry
        
        try:
            # Process available messages (non-blocking)
            while True:
                msg = self.master.recv_match(timeout=0.01)
                if msg is None:
                    break
                
                self._process_message(msg)
            
            # Return current telemetry
            return self.last_telemetry
            
        except Exception as e:
            logger.error(f"Error reading telemetry: {e}")
            return self.last_telemetry
    
    def _process_message(self, msg) -> None:
        """Process a MAVLink message and update telemetry cache.
        
        Args:
            msg: MAVLink message object
        """
        msg_type = msg.get_type()
        
        try:
            if msg_type == 'GPS_RAW_INT':
                # GPS data
                self.last_telemetry['lat'] = msg.lat / 1e7  # Convert from 1e7 degrees
                self.last_telemetry['lon'] = msg.lon / 1e7
                self.last_telemetry['altitude'] = msg.alt / 1000.0  # Convert from mm to m
                self.last_telemetry['gps_satellites'] = msg.satellites_visible
                self.last_telemetry['gps_fix_type'] = msg.fix_type
                
            elif msg_type == 'GLOBAL_POSITION_INT':
                # Global position (includes relative altitude)
                self.last_telemetry['relative_alt'] = msg.relative_alt / 1000.0  # Convert from mm to m
                if 'lat' not in self.last_telemetry:
                    self.last_telemetry['lat'] = msg.lat / 1e7
                if 'lon' not in self.last_telemetry:
                    self.last_telemetry['lon'] = msg.lon / 1e7
                if 'altitude' not in self.last_telemetry:
                    self.last_telemetry['altitude'] = msg.alt / 1000.0
                    
            elif msg_type == 'SYS_STATUS':
                # Battery and system status
                if msg.battery_remaining != 255:  # 255 = unknown
                    self.last_telemetry['battery_remaining'] = msg.battery_remaining
                if msg.voltage_battery != 65535:  # 65535 = unknown
                    self.last_telemetry['battery_voltage'] = msg.voltage_battery / 1000.0  # Convert from mV to V
                if msg.current_battery != -1:  # -1 = unknown
                    self.last_telemetry['battery_current'] = msg.current_battery / 100.0  # Convert from cA to A
                    
            elif msg_type == 'HEARTBEAT':
                # Vehicle state
                self.last_telemetry['armed'] = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                # Get mode string from autopilot type and custom mode
                try:
                    mode_string = mavutil.mode_string_v10(msg)
                    self.last_telemetry['mode'] = mode_string
                except Exception:
                    # Fallback: use custom_mode to determine mode
                    # This is a simplified fallback - may not work for all autopilots
                    self.last_telemetry['mode'] = f"MODE_{msg.custom_mode}"
                
            elif msg_type == 'ATTITUDE':
                # Attitude (roll, pitch, yaw)
                self.last_telemetry['roll'] = msg.roll
                self.last_telemetry['pitch'] = msg.pitch
                self.last_telemetry['yaw'] = msg.yaw
                self.last_telemetry['rollspeed'] = msg.rollspeed
                self.last_telemetry['pitchspeed'] = msg.pitchspeed
                self.last_telemetry['yawspeed'] = msg.yawspeed
                
            elif msg_type == 'VFR_HUD':
                # Velocity and heading
                self.last_telemetry['groundspeed'] = msg.groundspeed
                self.last_telemetry['airspeed'] = msg.airspeed
                self.last_telemetry['heading'] = msg.heading
                self.last_telemetry['throttle'] = msg.throttle
                if 'altitude' not in self.last_telemetry:
                    self.last_telemetry['altitude'] = msg.alt
                if 'climb' not in self.last_telemetry:
                    self.last_telemetry['climb'] = msg.climb
                    
            elif msg_type == 'RC_CHANNELS':
                # RC channels (for override detection)
                self.last_telemetry['rc_override_active'] = False  # Will be set by checking channels
                # Store channel values if needed
                if hasattr(msg, 'chan1_raw'):
                    self.last_telemetry['rc_channels'] = {
                        'roll': msg.chan1_raw if msg.chan1_raw > 0 else 1500,
                        'pitch': msg.chan2_raw if msg.chan2_raw > 0 else 1500,
                        'throttle': msg.chan3_raw if msg.chan3_raw > 0 else 1500,
                        'yaw': msg.chan4_raw if msg.chan4_raw > 0 else 1500
                    }
            
            # Update timestamp
            self.last_telemetry['timestamp'] = time.time()
            self.last_update = time.time()
            
        except Exception as e:
            logger.debug(f"Error processing {msg_type} message: {e}")
    
    def get_telemetry(self) -> Dict[str, Any]:
        """Get current telemetry data.
        
        Returns:
            Dictionary with telemetry data compatible with TelemetryCollector
        """
        # Read latest messages
        self.read_telemetry()
        
        # Return formatted telemetry
        return self.last_telemetry.copy()

