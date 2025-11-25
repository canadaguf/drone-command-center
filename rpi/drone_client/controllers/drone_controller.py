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
    
    def incremental_throttle_takeoff(self, 
                                     get_bottom_distance: callable,
                                     target_altitude: float = 1.5,
                                     max_altitude: float = 2.0,
                                     ground_threshold: float = 0.08,
                                     throttle_increment: int = 8,
                                     increment_interval: float = 0.1,
                                     max_timeout: float = 30.0) -> bool:
        """Incremental throttle takeoff using bottom ToF sensor.
        
        Starts at minimum throttle and increments by 1% until liftoff is detected,
        then continues to target altitude.
        
        Args:
            get_bottom_distance: Callable that returns current bottom sensor distance in meters
            target_altitude: Target altitude in meters (default 1.5m)
            max_altitude: Maximum allowed altitude in meters (default 2.0m)
            ground_threshold: Distance threshold to detect liftoff (default 0.08m = 8cm)
            throttle_increment: Throttle increment per step (default 8 = 1% of 800 range)
            increment_interval: Time between increments in seconds (default 0.1s)
            max_timeout: Maximum time for takeoff in seconds (default 30s)
            
        Returns:
            True if takeoff successful, False otherwise
        """
        if not self.is_connected():
            logger.error("Cannot takeoff: not connected")
            return False
        
        if not self.vehicle.armed:
            logger.error("Cannot takeoff: vehicle not armed")
            return False
        
        try:
            # Ensure GUIDED mode
            if self.vehicle.mode.name != 'GUIDED':
                if not self.set_mode('GUIDED'):
                    logger.error("Failed to set GUIDED mode for takeoff")
                    return False
            
            logger.info(f"Starting incremental throttle takeoff to {target_altitude}m (max {max_altitude}m)")
            
            # Start at minimum throttle
            current_throttle = 1100  # Minimum throttle value
            start_time = time.time()
            liftoff_detected = False
            
            # Get initial ground reading
            initial_distance = get_bottom_distance()
            if initial_distance is None:
                logger.warning("Cannot read bottom sensor - falling back to simple_takeoff")
                return self.simple_takeoff(target_altitude)
            
            logger.info(f"Initial ground distance: {initial_distance:.3f}m")
            
            # Incremental throttle rise until liftoff
            while not liftoff_detected and (time.time() - start_time) < max_timeout:
                # Set throttle
                self.send_rc_override(1500, 1500, 1500, current_throttle)
                
                # Check bottom sensor
                current_distance = get_bottom_distance()
                
                if current_distance is not None:
                    # Check if liftoff detected (distance increased beyond ground threshold)
                    if current_distance > ground_threshold:
                        liftoff_detected = True
                        logger.info(f"Liftoff detected! Distance: {current_distance:.3f}m, Throttle: {current_throttle}")
                        break
                
                # Increment throttle
                current_throttle += throttle_increment
                if current_throttle > 1900:
                    current_throttle = 1900
                    logger.warning("Maximum throttle reached before liftoff")
                    break
                
                time.sleep(increment_interval)
            
            if not liftoff_detected:
                logger.error("Liftoff not detected within timeout - aborting")
                self.clear_rc_override()
                return False
            
            # Now use velocity commands to reach target altitude
            logger.info(f"Ascending to target altitude {target_altitude}m...")
            
            # Use velocity commands for smooth ascent
            ascent_start_time = time.time()
            while (time.time() - ascent_start_time) < max_timeout:
                current_distance = get_bottom_distance()
                
                if current_distance is None:
                    logger.warning("Lost bottom sensor reading during ascent")
                    # Continue with velocity command
                    self.send_velocity_command(0.0, 0.0, 0.3, 0.0)  # Slow upward velocity
                    time.sleep(0.1)
                    continue
                
                # Check max altitude limit
                if current_distance > max_altitude:
                    logger.warning(f"Max altitude exceeded ({current_distance:.3f}m > {max_altitude}m) - stopping ascent")
                    self.send_velocity_command(0.0, 0.0, 0.0, 0.0)  # Stop
                    break
                
                # Check if target reached
                if current_distance >= target_altitude:
                    logger.info(f"Target altitude reached: {current_distance:.3f}m")
                    self.send_velocity_command(0.0, 0.0, 0.0, 0.0)  # Hover
                    break
                
                # Calculate upward velocity (proportional to remaining distance)
                remaining_distance = target_altitude - current_distance
                upward_velocity = min(0.5, remaining_distance * 0.5)  # Max 0.5 m/s
                
                self.send_velocity_command(0.0, 0.0, upward_velocity, 0.0)
                time.sleep(0.1)
            
            # Clear RC override and switch to velocity control
            self.clear_rc_override()
            
            # Final hover command
            self.send_velocity_command(0.0, 0.0, 0.0, 0.0)
            
            logger.info("Incremental throttle takeoff completed")
            return True
            
        except Exception as e:
            logger.error(f"Error during incremental throttle takeoff: {e}")
            import traceback
            logger.error(traceback.format_exc())
            self.clear_rc_override()
            return False
    
    def simple_land(self) -> bool:
        """Land the vehicle (deprecated - use incremental_throttle_land instead).
        
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
    
    def incremental_throttle_land(self,
                                  get_bottom_distance: callable,
                                  ground_threshold: float = 0.08,
                                  throttle_decrement: int = 4,
                                  decrement_interval: float = 0.15,
                                  min_throttle: int = 1100,
                                  max_timeout: float = 60.0) -> bool:
        """Incremental throttle landing using bottom ToF sensor.
        
        Slowly reduces throttle while monitoring bottom sensor to detect ground contact.
        This is safer than autonomous landing mode as it provides precise control.
        
        Args:
            get_bottom_distance: Callable that returns current bottom sensor distance in meters
            ground_threshold: Distance threshold to detect ground contact (default 0.08m = 8cm)
            throttle_decrement: Throttle decrement per step (default 4 = 0.5% of 800 range, slower than takeoff)
            decrement_interval: Time between decrements in seconds (default 0.15s, slower than takeoff)
            min_throttle: Minimum throttle to maintain (default 1100, absolute minimum)
            max_timeout: Maximum time for landing in seconds (default 60s, longer than takeoff)
            
        Returns:
            True if landing successful, False otherwise
        """
        if not self.is_connected():
            logger.error("Cannot land: not connected")
            return False
        
        if not self.vehicle.armed:
            logger.warning("Vehicle already disarmed")
            return True
        
        try:
            # Ensure GUIDED mode
            if self.vehicle.mode.name != 'GUIDED':
                if not self.set_mode('GUIDED'):
                    logger.error("Failed to set GUIDED mode for landing")
                    return False
            
            logger.info("Starting incremental throttle landing using bottom sensor")
            
            # Get current height
            initial_distance = get_bottom_distance()
            if initial_distance is None:
                logger.error("Cannot read bottom sensor - landing aborted")
                return False
            
            logger.info(f"Initial height: {initial_distance:.3f}m")
            
            # First, use velocity commands to descend smoothly
            # This is safer than immediately switching to RC override
            logger.info("Phase 1: Controlled descent using velocity commands...")
            descent_start_time = time.time()
            descent_phase_timeout = max_timeout * 0.7  # Use 70% of timeout for descent
            
            while (time.time() - descent_start_time) < descent_phase_timeout:
                current_distance = get_bottom_distance()
                
                if current_distance is None:
                    logger.warning("Lost bottom sensor reading during descent - continuing carefully")
                    # Very slow descent if sensor unavailable
                    self.send_velocity_command(0.0, 0.0, -0.2, 0.0)  # 0.2 m/s down
                    time.sleep(0.2)
                    continue
                
                # Check if close to ground (within 30cm)
                if current_distance <= 0.30:
                    logger.info(f"Close to ground ({current_distance:.3f}m) - switching to throttle control")
                    break
                
                # Calculate downward velocity (proportional to height, max 0.5 m/s)
                # Slower descent when closer to ground
                if current_distance > 1.0:
                    downward_velocity = -0.5  # Max descent rate
                elif current_distance > 0.5:
                    downward_velocity = -0.3  # Medium descent rate
                else:
                    downward_velocity = -0.15  # Slow descent when close
            
                self.send_velocity_command(0.0, 0.0, downward_velocity, 0.0)
                time.sleep(0.15)  # Update every 150ms
            
            # Phase 2: Switch to RC override for precise throttle control near ground
            logger.info("Phase 2: Precise throttle control near ground...")
            
            # Get current throttle from vehicle (if available) or estimate
            # We'll start from a safe hover throttle and reduce gradually
            current_throttle = 1500  # Start at neutral/mid-range throttle
            landing_start_time = time.time()
            ground_contact_detected = False
            
            # Small delay to stabilize after velocity command
            time.sleep(0.2)
            
            while not ground_contact_detected and (time.time() - landing_start_time) < (max_timeout * 0.3):
                # Read bottom sensor
                current_distance = get_bottom_distance()
                
                if current_distance is None:
                    logger.warning("Lost bottom sensor reading - using conservative throttle reduction")
                    # Very slow throttle reduction if sensor unavailable
                    current_throttle = max(min_throttle, current_throttle - throttle_decrement // 2)
                    self.send_rc_override(1500, 1500, 1500, current_throttle)
                    time.sleep(decrement_interval * 1.5)  # Slower updates
                    continue
                
                # Check for ground contact
                if current_distance <= ground_threshold:
                    ground_contact_detected = True
                    logger.info(f"Ground contact detected! Distance: {current_distance:.3f}m")
                    break
                
                # Reduce throttle gradually
                # Reduce faster when higher, slower when closer to ground
                if current_distance > 0.20:
                    # Still relatively high - can reduce faster
                    throttle_reduction = throttle_decrement
                elif current_distance > 0.12:
                    # Getting close - reduce slower
                    throttle_reduction = throttle_decrement // 2
                else:
                    # Very close - reduce very slowly
                    throttle_reduction = throttle_decrement // 4
                
                current_throttle = max(min_throttle, current_throttle - throttle_reduction)
                
                # Set throttle via RC override
                self.send_rc_override(1500, 1500, 1500, current_throttle)
                
                logger.debug(f"Height: {current_distance:.3f}m, Throttle: {current_throttle}")
                
                time.sleep(decrement_interval)
            
            if not ground_contact_detected:
                logger.warning("Ground contact not detected within timeout - reducing throttle to minimum")
                # Force minimum throttle
                self.send_rc_override(1500, 1500, 1500, min_throttle)
                time.sleep(1.0)  # Wait a moment
            
            # Phase 3: Final disarm sequence
            logger.info("Phase 3: Finalizing landing...")
            
            # Set throttle to minimum
            self.send_rc_override(1500, 1500, 1500, min_throttle)
            time.sleep(0.5)
            
            # Clear RC override and disarm
            self.clear_rc_override()
            time.sleep(0.5)
            
            # Disarm the vehicle
            logger.info("Disarming motors...")
            self.vehicle.armed = False
            
            # Wait for disarm to complete
            timeout = 5
            start_time = time.time()
            while self.vehicle.armed and (time.time() - start_time) < timeout:
                time.sleep(0.1)
            
            if not self.vehicle.armed:
                logger.info("Landing completed successfully - vehicle disarmed")
                return True
            else:
                logger.warning("Landing completed but vehicle still armed")
                return True  # Still consider it successful if we're on the ground
            
        except Exception as e:
            logger.error(f"Error during incremental throttle landing: {e}")
            import traceback
            logger.error(traceback.format_exc())
            # Try to clear RC override and set safe mode
            try:
                self.clear_rc_override()
                self.set_mode('LOITER')
            except:
                pass
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

