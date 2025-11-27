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
            # Build connection string (baud rate passed as separate parameter)
            conn_str = self.connection_string
            
            logger.info(f"Connecting to vehicle at {conn_str} (baud={self.baud})...")
            # Pass baud rate as keyword argument, not in connection string
            self.vehicle = connect(conn_str, baud=self.baud, wait_ready=['autopilot_version'])
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
            
            # Set mode to ALT_HOLD (works indoors without GPS)
            if self.vehicle.mode.name != 'ALT_HOLD':
                logger.info("Setting mode to ALT_HOLD for arming")
                self.vehicle.mode = VehicleMode("ALT_HOLD")
                while self.vehicle.mode.name != 'ALT_HOLD':
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
            
            # Ensure ALT_HOLD mode
            if self.vehicle.mode.name != 'ALT_HOLD':
                if not self.set_mode('ALT_HOLD'):
                    logger.error("Failed to set ALT_HOLD mode for takeoff")
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
                                     throttle_increment: int = 3,
                                     increment_interval: float = 0.25,
                                     max_timeout: float = 60.0) -> bool:
        """Smooth incremental throttle takeoff using bottom ToF sensor.
        
        Uses slow, controlled throttle increments with continuous sensor feedback.
        After liftoff, continues with throttle-based altitude control until target reached.
        
        Args:
            get_bottom_distance: Callable that returns current bottom sensor distance in meters
            target_altitude: Target altitude in meters (default 1.5m)
            max_altitude: Maximum allowed altitude in meters (default 2.0m)
            ground_threshold: Distance threshold to detect liftoff (default 0.08m = 8cm)
            throttle_increment: Throttle increment per step (default 3 = 0.375% of 800 range, very slow)
            increment_interval: Time between increments in seconds (default 0.25s, slower)
            max_timeout: Maximum time for takeoff in seconds (default 60s)
            
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
            # Ensure ALT_HOLD mode
            if self.vehicle.mode.name != 'ALT_HOLD':
                if not self.set_mode('ALT_HOLD'):
                    logger.error("Failed to set ALT_HOLD mode for takeoff")
                    return False
            
            logger.info(f"Starting smooth incremental throttle takeoff to {target_altitude}m (max {max_altitude}m)")
            
            # Start at minimum throttle
            current_throttle = 1100  # Minimum throttle value
            start_time = time.time()
            liftoff_detected = False
            consecutive_sensor_failures = 0
            max_sensor_failures = 5  # Abort if sensor fails 5 times in a row
            
            # Get initial ground reading with retry
            initial_distance = None
            for attempt in range(5):
                initial_distance = get_bottom_distance()
                if initial_distance is not None:
                    break
                time.sleep(0.1)
            
            if initial_distance is None:
                logger.error("Cannot read bottom sensor - aborting takeoff")
                return False
            
            logger.info(f"Initial ground distance: {initial_distance:.3f}m")
            
            # Phase 1: Incremental throttle rise until liftoff (slow and controlled)
            logger.info("Phase 1: Slow throttle increase until liftoff...")
            logger.info(f"Starting throttle: {current_throttle}, increment: {throttle_increment} per {increment_interval}s")
            last_rc_send_time = 0
            rc_send_interval = 0.1  # Send RC override every 100ms to maintain control
            
            while not liftoff_detected and (time.time() - start_time) < max_timeout:
                current_time = time.time()
                
                # Send RC override frequently to maintain control (ArduPilot requires frequent updates)
                if (current_time - last_rc_send_time) >= rc_send_interval:
                    if not self.send_rc_override(1500, 1500, 1500, current_throttle):
                        logger.warning(f"Failed to send RC override at throttle {current_throttle}")
                    last_rc_send_time = current_time
                
                # Wait a bit for throttle to take effect
                time.sleep(increment_interval * 0.5)
                
                # Check bottom sensor (critical - must always read)
                current_distance = get_bottom_distance()
                
                if current_distance is None:
                    consecutive_sensor_failures += 1
                    logger.warning(f"Sensor read failed ({consecutive_sensor_failures}/{max_sensor_failures})")
                    if consecutive_sensor_failures >= max_sensor_failures:
                        logger.error("Too many sensor failures - aborting takeoff")
                        self.clear_rc_override()
                        return False
                    # Don't increment throttle if sensor failed
                    time.sleep(increment_interval)
                    continue
                
                # Reset sensor failure counter on successful read
                consecutive_sensor_failures = 0
                
                # Check if liftoff detected (distance increased beyond ground threshold)
                if current_distance > ground_threshold:
                    liftoff_detected = True
                    logger.info(f"Liftoff detected! Distance: {current_distance:.3f}m, Throttle: {current_throttle}")
                    break
                
                # Increment throttle slowly
                current_throttle += throttle_increment
                if current_throttle > 1900:
                    current_throttle = 1900
                    logger.warning("Maximum throttle reached before liftoff")
                    # Give it a moment at max throttle
                    time.sleep(increment_interval * 2)
                    break
                
                time.sleep(increment_interval)
            
            if not liftoff_detected:
                logger.error("Liftoff not detected within timeout - aborting")
                self.clear_rc_override()
                return False
            
            # Phase 2: Controlled ascent using throttle with sensor feedback (PID-like control)
            logger.info(f"Phase 2: Controlled ascent to target altitude {target_altitude}m...")
            ascent_start_time = time.time()
            last_distance = current_distance
            last_error = target_altitude - current_distance
            integral_error = 0.0
            
            # PID-like constants for smooth ascent
            kp = 15.0  # Proportional gain (throttle per meter error)
            ki = 2.0   # Integral gain (small, to prevent windup)
            kd = 5.0   # Derivative gain (dampening)
            
            # Base throttle for hover (calibrated during liftoff)
            hover_throttle = current_throttle
            
            while (time.time() - ascent_start_time) < max_timeout:
                # Always read sensor - critical for safety
                current_distance = get_bottom_distance()
                
                if current_distance is None:
                    consecutive_sensor_failures += 1
                    logger.warning(f"Sensor read failed during ascent ({consecutive_sensor_failures}/{max_sensor_failures})")
                    if consecutive_sensor_failures >= max_sensor_failures:
                        logger.error("Too many sensor failures during ascent - stopping and holding")
                        # Hold current throttle
                        self.send_rc_override(1500, 1500, 1500, current_throttle)
                        time.sleep(0.5)
                        # Try to stabilize
                        self.send_velocity_command(0.0, 0.0, 0.0, 0.0)
                        return False
                    # Hold current throttle if sensor fails
                    time.sleep(0.2)
                    continue
                
                # Reset sensor failure counter
                consecutive_sensor_failures = 0
                
                # Safety check: max altitude limit
                if current_distance > max_altitude:
                    logger.warning(f"Max altitude exceeded ({current_distance:.3f}m > {max_altitude}m) - reducing throttle")
                    # Reduce throttle significantly
                    current_throttle = max(1100, current_throttle - throttle_increment * 5)
                    self.send_rc_override(1500, 1500, 1500, current_throttle)
                    time.sleep(0.3)
                    continue
                
                # Check if target reached (with tolerance)
                altitude_tolerance = 0.05  # 5cm tolerance
                if abs(current_distance - target_altitude) < altitude_tolerance:
                    logger.info(f"Target altitude reached: {current_distance:.3f}m (target: {target_altitude}m)")
                    # Hold throttle at current level
                    self.send_rc_override(1500, 1500, 1500, current_throttle)
                    time.sleep(0.5)
                    break
                
                # PID-like control for smooth ascent
                error = target_altitude - current_distance
                integral_error += error * 0.2  # Integrate over 0.2s intervals
                integral_error = max(-0.5, min(0.5, integral_error))  # Limit integral windup
                derivative_error = (error - last_error) / 0.2  # Rate of change
                
                # Calculate throttle adjustment
                throttle_adjustment = (kp * error + ki * integral_error + kd * derivative_error) * 0.1
                
                # Update throttle smoothly
                current_throttle += int(throttle_adjustment)
                current_throttle = max(1100, min(1900, current_throttle))  # Clamp to valid range
                
                # Set throttle
                self.send_rc_override(1500, 1500, 1500, current_throttle)
                
                # Update for next iteration
                last_distance = current_distance
                last_error = error
                
                # Log progress every 0.5m
                if int(current_distance * 2) != int(last_distance * 2):
                    logger.info(f"Ascending: {current_distance:.3f}m / {target_altitude}m, throttle: {current_throttle}")
                
                time.sleep(0.2)  # Update every 200ms for smooth control
            
            # Phase 3: Transition to velocity control (gradual)
            logger.info("Phase 3: Transitioning to velocity control...")
            
            # Clear RC override gradually (reduce throttle slightly first)
            for i in range(3):
                current_throttle = max(1100, current_throttle - throttle_increment)
                self.send_rc_override(1500, 1500, 1500, current_throttle)
                time.sleep(0.2)
            
            # Clear RC override
            self.clear_rc_override()
            time.sleep(0.3)  # Give time for mode transition
            
            # Final hover command using velocity control
            self.send_velocity_command(0.0, 0.0, 0.0, 0.0)
            
            logger.info("Smooth incremental throttle takeoff completed successfully")
            return True
            
        except Exception as e:
            logger.error(f"Error during incremental throttle takeoff: {e}")
            import traceback
            logger.error(traceback.format_exc())
            # Emergency: clear RC override and stop
            try:
                self.clear_rc_override()
                self.send_velocity_command(0.0, 0.0, 0.0, 0.0)
            except:
                pass
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
            # Ensure ALT_HOLD mode
            if self.vehicle.mode.name != 'ALT_HOLD':
                if not self.set_mode('ALT_HOLD'):
                    logger.error("Failed to set ALT_HOLD mode for landing")
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
        """Send velocity command in ALT_HOLD mode.
        
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
            # Ensure ALT_HOLD mode
            if self.vehicle.mode.name != 'ALT_HOLD':
                if not self.set_mode('ALT_HOLD'):
                    logger.error("Failed to set ALT_HOLD mode for velocity command")
                    return False
            
            # Send velocity command via MAVLink (vehicle.velocity is read-only)
            # Convert from body frame (forward, right, up) to NED frame (North, East, Down)
            
            # Get current heading for conversion
            current_heading = math.radians(self.vehicle.heading)
            
            # Convert body frame to NED frame
            # Forward (vx) -> North component
            # Right (vy) -> East component
            # Up (vz) -> Down component (negative because NED uses Down as positive)
            velocity_north = vx * math.cos(current_heading) - vy * math.sin(current_heading)
            velocity_east = vx * math.sin(current_heading) + vy * math.cos(current_heading)
            velocity_down = -vz  # NED uses down as positive
            
            # Type mask: ignore position (bits 0-2), use velocity (bits 3-5), 
            # ignore acceleration (bits 6-8), ignore force (bit 9), ignore yaw (bit 10), use yaw_rate (bit 11)
            # Binary: 0b0000111111000111 = 0x3F07 = 16135
            type_mask = 0b0000111111000111  # Ignore position, acceleration, force, yaw; use velocity and yaw_rate
            
            # Send velocity command via MAVLink SET_POSITION_TARGET_LOCAL_NED
            # Get target system and component from vehicle's master connection
            target_system = self.vehicle._master.target_system if hasattr(self.vehicle, '_master') else 1
            target_component = self.vehicle._master.target_component if hasattr(self.vehicle, '_master') else 1
            
            msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
                0,  # time_boot_ms (not used)
                target_system,
                target_component,
                8,  # frame (MAV_FRAME_LOCAL_NED)
                type_mask,
                0, 0, 0,  # x, y, z (ignored - position)
                velocity_north, velocity_east, velocity_down,  # vx, vy, vz (velocity in NED frame)
                0, 0, 0,  # afx, afy, afz (ignored - acceleration)
                0,  # yaw (ignored)
                yaw_rate  # yaw_rate (rad/s)
            )
            self.vehicle.send_mavlink(msg)
            
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
            # Get target system and component from vehicle's master connection
            target_system = self.vehicle._master.target_system if hasattr(self.vehicle, '_master') else 1
            target_component = self.vehicle._master.target_component if hasattr(self.vehicle, '_master') else 1
            
            # Use MAVLink message for yaw rate
            # MAV_CMD_DO_SET_ROI_LOCATION or use velocity_yaw_rate in velocity_ned
            # For ArduPilot, we can use SET_POSITION_TARGET_LOCAL_NED with velocity_yaw_rate
            msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
                0,  # time_boot_ms
                target_system,
                target_component,
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
    
    def send_distance_sensor(self, distance_m: float, sensor_id: int = 0, 
                              orientation: int = 25, min_distance_m: float = 0.04, 
                              max_distance_m: float = 4.0) -> bool:
        """Send distance sensor data to ArduPilot EKF via MAVLink DISTANCE_SENSOR message.
        
        This allows ArduPilot's EKF to fuse ToF sensor data for better altitude estimation.
        
        Args:
            distance_m: Distance reading in meters
            sensor_id: Sensor ID (0-7, use 0 for downward, 1 for forward)
            orientation: Sensor orientation (25=downward, 0=forward)
            min_distance_m: Minimum measurable distance in meters
            max_distance_m: Maximum measurable distance in meters
            
        Returns:
            True if message sent successfully, False otherwise
        """
        if not self.is_connected():
            return False
        
        try:
            # Get target system and component
            target_system = self.vehicle._master.target_system if hasattr(self.vehicle, '_master') else 1
            target_component = self.vehicle._master.target_component if hasattr(self.vehicle, '_master') else 1
            
            # Convert meters to millimeters (MAVLink uses mm)
            distance_mm = int(distance_m * 1000)
            min_distance_mm = int(min_distance_m * 1000)
            max_distance_mm = int(max_distance_m * 1000)
            
            # Clamp distance to valid range
            distance_mm = max(min_distance_mm, min(max_distance_mm, distance_mm))
            
            # Ensure values are within valid range for unsigned integers
            distance_mm = max(0, min(4294967295, distance_mm))
            min_distance_mm = max(0, min(4294967295, min_distance_mm))
            max_distance_mm = max(0, min(4294967295, max_distance_mm))
            
            # MAV_DISTANCE_SENSOR_LASER = 1 (for ToF sensors)
            sensor_type = 1
            
            # Get time_boot_ms from vehicle if available, otherwise use 0
            # time_boot_ms should be milliseconds since boot, not absolute time
            try:
                time_boot_ms = self.vehicle.time_boot_ms if hasattr(self.vehicle, 'time_boot_ms') else 0
            except:
                time_boot_ms = 0
            
            # Ensure time_boot_ms is within valid unsigned 32-bit range
            time_boot_ms = max(0, min(4294967295, int(time_boot_ms)))
            
            # Create DISTANCE_SENSOR message
            msg = self.vehicle.message_factory.distance_sensor_encode(
                time_boot_ms,             # time_boot_ms (milliseconds since boot)
                min_distance_mm,          # min_distance (mm)
                max_distance_mm,          # max_distance (mm)
                distance_mm,             # current_distance (mm)
                sensor_type,             # type (MAV_DISTANCE_SENSOR_LASER)
                sensor_id,               # id (0-7)
                orientation,             # orientation (25=downward, 0=forward)
                0                        # covariance (0 = unknown/ignored)
            )
            
            self.vehicle.send_mavlink(msg)
            return True
            
        except Exception as e:
            logger.error(f"Error sending distance sensor data: {e}")
            import traceback
            logger.debug(traceback.format_exc())
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

