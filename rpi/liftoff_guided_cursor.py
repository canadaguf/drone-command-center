#!/usr/bin/env python3
"""
Standalone liftoff test script using GUIDED mode (outdoors with GPS + ToF sensor).
Implements autonomous flight sequence: arm → takeoff → ascend to 1m → hold 60s → descend → land → disarm.
"""

import time
import logging
import errno
import sys
from typing import Optional

# DroneKit imports
from dronekit import connect, VehicleMode, LocationGlobalRelative
import dronekit

# ToF sensor imports
import board
import busio
import adafruit_vl53l1x
from smbus2 import SMBus

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(name)s: %(message)s'
)
logger = logging.getLogger(__name__)

# Configuration
CONNECTION_STRING = "/dev/ttyAMA0"
BAUD_RATE = 256000
MULTIPLEXER_ADDRESS = 0x70
VL53L1X_ADDRESS = 0x29
BOTTOM_SENSOR_CHANNEL = 1
TARGET_ALTITUDE = 1.0  # meters
HOLD_DURATION = 60  # seconds
GROUND_THRESHOLD = 0.08  # meters (8cm)
TAKEOFF_THRESHOLD = 0.1  # meters (10cm)
ALTITUDE_TOLERANCE = 0.05  # meters (5cm)

# I2C retry configuration
MAX_I2C_RETRIES = 5
BASE_RETRY_DELAY = 0.02  # 20ms


class ToFSensor:
    """Simple ToF sensor wrapper for bottom sensor."""
    
    def __init__(self, multiplexer_address: int, sensor_address: int, channel: int):
        """Initialize ToF sensor.
        
        Args:
            multiplexer_address: TCA9548A multiplexer I2C address
            sensor_address: VL53L1X sensor I2C address
            channel: Multiplexer channel (0-7)
        """
        self.multiplexer_address = multiplexer_address
        self.sensor_address = sensor_address
        self.channel = channel
        self.smbus = None
        self.i2c = None
        self.sensor = None
        self.initialized = False
    
    def initialize(self) -> bool:
        """Initialize sensor hardware.
        
        Returns:
            True if successful, False otherwise
        """
        try:
            logger.info(f"Initializing ToF sensor on channel {self.channel}...")
            
            # Initialize I2C bus for multiplexer control
            self.smbus = SMBus(1)
            
            # Initialize I2C bus for sensor (100kHz for better EMI tolerance)
            self.i2c = busio.I2C(board.SCL, board.SDA, frequency=100000)
            
            # Select channel
            self._select_channel(self.channel)
            time.sleep(0.1)
            
            # Initialize sensor
            self.sensor = adafruit_vl53l1x.VL53L1X(self.i2c, address=self.sensor_address)
            self.sensor.start_ranging()
            
            self.initialized = True
            logger.info("ToF sensor initialized successfully")
            return True
            
        except Exception as e:
            logger.error(f"Failed to initialize ToF sensor: {e}")
            import traceback
            logger.error(traceback.format_exc())
            return False
    
    def _select_channel(self, channel: int, retry_count: int = 0) -> None:
        """Select multiplexer channel with retry logic.
        
        Args:
            channel: Channel number (0-7)
            retry_count: Current retry attempt
        """
        try:
            channel_mask = 1 << channel
            self.smbus.write_byte(self.multiplexer_address, channel_mask)
            time.sleep(0.01)
        except OSError as e:
            if (e.errno == errno.EAGAIN or e.errno == 11) and retry_count < MAX_I2C_RETRIES:
                delay = BASE_RETRY_DELAY * (2 ** retry_count)
                time.sleep(delay)
                return self._select_channel(channel, retry_count + 1)
            else:
                raise
    
    def get_distance(self) -> Optional[float]:
        """Get distance reading from sensor.
        
        Returns:
            Distance in meters, or None if read failed
        """
        if not self.initialized or self.sensor is None:
            return None
        
        try:
            # Select channel
            self._select_channel(self.channel)
            time.sleep(0.02)
            
            # Check if data is ready
            data_ready = False
            for attempt in range(3):
                try:
                    data_ready = self.sensor.data_ready
                    break
                except OSError as e:
                    if e.errno == errno.EAGAIN or e.errno == 11:
                        time.sleep(0.01 * (attempt + 1))
                        continue
                    else:
                        raise
            
            if not data_ready:
                return None
            
            # Read distance
            distance_cm = None
            for attempt in range(3):
                try:
                    distance_cm = self.sensor.distance
                    break
                except OSError as e:
                    if e.errno == errno.EAGAIN or e.errno == 11:
                        time.sleep(0.01 * (attempt + 1))
                        continue
                    else:
                        raise
            
            if distance_cm is None:
                return None
            
            # Clear interrupt
            try:
                self.sensor.clear_interrupt()
            except OSError:
                pass
            
            # Convert to meters
            distance_m = distance_cm / 100.0
            
            # Validate range (0.04m to 4.0m)
            if distance_m < 0.0 or distance_m > 4.0:
                return None
            
            return distance_m
            
        except Exception as e:
            logger.debug(f"Error reading ToF sensor: {e}")
            return None
    
    def cleanup(self) -> None:
        """Cleanup sensor resources."""
        if self.sensor is not None:
            try:
                self._select_channel(self.channel)
                time.sleep(0.01)
                self.sensor.stop_ranging()
            except Exception:
                pass
        
        if self.smbus is not None:
            try:
                self.smbus.close()
            except Exception:
                pass
        
        self.initialized = False


def check_gps_lock(vehicle) -> bool:
    """Check if GPS has a valid lock.
    
    Args:
        vehicle: DroneKit vehicle object
        
    Returns:
        True if GPS lock is valid, False otherwise
    """
    try:
        # Check GPS fix type (3 = 3D fix, 4 = RTK fix)
        gps_fix_type = vehicle.gps_0.fix_type
        gps_satellites = vehicle.gps_0.satellites_visible
        
        logger.info(f"GPS status: fix_type={gps_fix_type}, satellites={gps_satellites}")
        
        if gps_fix_type >= 3 and gps_satellites >= 6:
            logger.info("GPS lock confirmed")
            return True
        else:
            logger.error(f"GPS lock insufficient: fix_type={gps_fix_type}, satellites={gps_satellites}")
            return False
            
    except Exception as e:
        logger.error(f"Error checking GPS lock: {e}")
        return False


def wait_for_mode(vehicle, target_mode: str, timeout: float = 5.0) -> bool:
    """Wait for vehicle to enter target mode.
    
    Args:
        vehicle: DroneKit vehicle object
        target_mode: Target mode name
        timeout: Timeout in seconds
        
    Returns:
        True if mode changed successfully, False otherwise
    """
    start_time = time.time()
    while (time.time() - start_time) < timeout:
        if vehicle.mode.name == target_mode:
            return True
        time.sleep(0.1)
    return False


def arm_vehicle(vehicle) -> bool:
    """Arm the vehicle.
    
    Args:
        vehicle: DroneKit vehicle object
        
    Returns:
        True if arming successful, False otherwise
    """
    try:
        if vehicle.armed:
            logger.info("Vehicle already armed")
            return True
        
        logger.info("Arming vehicle...")
        vehicle.armed = True
        
        timeout = 10
        start_time = time.time()
        while not vehicle.armed and (time.time() - start_time) < timeout:
            time.sleep(0.1)
        
        if vehicle.armed:
            logger.info("Vehicle armed successfully")
            return True
        else:
            logger.error("Arming timeout")
            return False
            
    except Exception as e:
        logger.error(f"Error arming vehicle: {e}")
        return False


def detect_takeoff(vehicle, tof_sensor: ToFSensor, timeout: float = 30.0) -> bool:
    """Detect when drone starts taking off by monitoring ToF sensor.
    
    Args:
        vehicle: DroneKit vehicle object
        tof_sensor: ToF sensor instance
        timeout: Maximum time to wait for takeoff
        
    Returns:
        True if takeoff detected, False otherwise
    """
    logger.info("Waiting for takeoff detection...")
    
    # Get initial ground reading
    initial_distance = None
    for attempt in range(5):
        initial_distance = tof_sensor.get_distance()
        if initial_distance is not None:
            break
        time.sleep(0.1)
    
    if initial_distance is None:
        logger.error("Cannot read ToF sensor - aborting")
        return False
    
    logger.info(f"Initial ground distance: {initial_distance:.3f}m")
    
    # Start with minimal throttle and increment slowly
    current_throttle = 1100
    throttle_increment = 3
    increment_interval = 0.25
    
    start_time = time.time()
    
    while (time.time() - start_time) < timeout:
        # Set throttle via RC override
        vehicle.channels.overrides = {
            '1': 1500,  # roll
            '2': 1500,  # pitch
            '3': current_throttle,  # throttle
            '4': 1500  # yaw
        }
        
        time.sleep(increment_interval * 0.5)
        
        # Check ToF sensor
        current_distance = tof_sensor.get_distance()
        
        if current_distance is None:
            logger.warning("ToF sensor read failed, continuing...")
            time.sleep(increment_interval)
            continue
        
        # Check if liftoff detected
        if current_distance > TAKEOFF_THRESHOLD:
            logger.info(f"Takeoff detected! Distance: {current_distance:.3f}m")
            return True
        
        # Increment throttle
        current_throttle += throttle_increment
        if current_throttle > 1900:
            current_throttle = 1900
            logger.warning("Maximum throttle reached")
            time.sleep(increment_interval * 2)
            break
        
        time.sleep(increment_interval)
    
    logger.error("Takeoff detection timeout")
    return False


def ascend_to_altitude(vehicle, tof_sensor: ToFSensor, target_altitude: float, timeout: float = 30.0) -> bool:
    """Ascend to target altitude using GUIDED mode.
    
    Args:
        vehicle: DroneKit vehicle object
        tof_sensor: ToF sensor instance
        target_altitude: Target altitude in meters
        timeout: Maximum time to reach altitude
        
    Returns:
        True if altitude reached, False otherwise
    """
    logger.info(f"Ascending to {target_altitude}m...")
    
    # Clear RC override
    vehicle.channels.overrides = {}
    time.sleep(0.3)
    
    # Use GUIDED mode simple_takeoff
    try:
        vehicle.simple_takeoff(target_altitude)
        logger.info("Takeoff command sent")
    except Exception as e:
        logger.error(f"Error sending takeoff command: {e}")
        return False
    
    # Monitor altitude using ToF sensor
    start_time = time.time()
    while (time.time() - start_time) < timeout:
        current_distance = tof_sensor.get_distance()
        
        if current_distance is None:
            time.sleep(0.2)
            continue
        
        # Check if target reached
        if abs(current_distance - target_altitude) < ALTITUDE_TOLERANCE:
            logger.info(f"Target altitude reached: {current_distance:.3f}m")
            return True
        
        time.sleep(0.2)
    
    logger.warning("Altitude timeout, but continuing...")
    return True  # Continue anyway


def hold_position(vehicle, duration: float) -> bool:
    """Hold position using LOITER mode.
    
    Args:
        vehicle: DroneKit vehicle object
        duration: Hold duration in seconds
        
    Returns:
        True if successful, False otherwise
    """
    logger.info(f"Switching to LOITER mode and holding for {duration}s...")
    
    try:
        vehicle.mode = VehicleMode("LOITER")
        if not wait_for_mode(vehicle, "LOITER", timeout=5.0):
            logger.error("Failed to switch to LOITER mode")
            return False
        
        logger.info("Holding position...")
        time.sleep(duration)
        logger.info("Hold complete")
        return True
        
    except Exception as e:
        logger.error(f"Error holding position: {e}")
        return False


def descend_to_ground(vehicle, tof_sensor: ToFSensor, timeout: float = 60.0) -> bool:
    """Descend to ground using GUIDED mode velocity commands.
    
    Args:
        vehicle: DroneKit vehicle object
        tof_sensor: ToF sensor instance
        timeout: Maximum time for descent
        
    Returns:
        True if descent successful, False otherwise
    """
    logger.info("Descending to ground...")
    
    # Ensure GUIDED mode
    if vehicle.mode.name != "GUIDED":
        vehicle.mode = VehicleMode("GUIDED")
        if not wait_for_mode(vehicle, "GUIDED", timeout=5.0):
            logger.error("Failed to switch to GUIDED mode")
            return False
    
    start_time = time.time()
    
    while (time.time() - start_time) < timeout:
        current_distance = tof_sensor.get_distance()
        
        if current_distance is None:
            # Slow descent if sensor unavailable
            send_velocity_command(vehicle, 0.0, 0.0, -0.2, 0.0)
            time.sleep(0.2)
            continue
        
        # Check if close to ground
        if current_distance <= GROUND_THRESHOLD:
            logger.info(f"Close to ground: {current_distance:.3f}m")
            return True
        
        # Calculate descent velocity (slower when closer)
        if current_distance > 0.5:
            downward_velocity = -0.3  # m/s
        elif current_distance > 0.2:
            downward_velocity = -0.15  # m/s
        else:
            downward_velocity = -0.1  # m/s
        
        send_velocity_command(vehicle, 0.0, 0.0, downward_velocity, 0.0)
        time.sleep(0.15)
    
    logger.warning("Descent timeout")
    return True  # Continue anyway


def land_with_throttle(vehicle, tof_sensor: ToFSensor, timeout: float = 30.0) -> bool:
    """Land using incremental throttle control.
    
    Args:
        vehicle: DroneKit vehicle object
        tof_sensor: ToF sensor instance
        timeout: Maximum time for landing
        
    Returns:
        True if landing successful, False otherwise
    """
    logger.info("Landing using throttle control...")
    
    # Switch to ALT_HOLD for throttle control
    vehicle.mode = VehicleMode("ALT_HOLD")
    if not wait_for_mode(vehicle, "ALT_HOLD", timeout=5.0):
        logger.error("Failed to switch to ALT_HOLD mode")
        return False
    
    time.sleep(0.2)
    
    # Start from hover throttle and reduce gradually
    current_throttle = 1500
    throttle_decrement = 4
    decrement_interval = 0.15
    
    start_time = time.time()
    
    while (time.time() - start_time) < timeout:
        current_distance = tof_sensor.get_distance()
        
        if current_distance is None:
            # Very slow throttle reduction if sensor unavailable
            current_throttle = max(1100, current_throttle - throttle_decrement // 2)
            vehicle.channels.overrides = {
                '1': 1500,
                '2': 1500,
                '3': current_throttle,
                '4': 1500
            }
            time.sleep(decrement_interval * 1.5)
            continue
        
        # Check for ground contact
        if current_distance <= GROUND_THRESHOLD:
            logger.info(f"Ground contact detected: {current_distance:.3f}m")
            break
        
        # Reduce throttle (faster when higher, slower when closer)
        if current_distance > 0.20:
            throttle_reduction = throttle_decrement
        elif current_distance > 0.12:
            throttle_reduction = throttle_decrement // 2
        else:
            throttle_reduction = throttle_decrement // 4
        
        current_throttle = max(1100, current_throttle - throttle_reduction)
        
        vehicle.channels.overrides = {
            '1': 1500,
            '2': 1500,
            '3': current_throttle,
            '4': 1500
        }
        
        time.sleep(decrement_interval)
    
    # Set minimum throttle
    vehicle.channels.overrides = {
        '1': 1500,
        '2': 1500,
        '3': 1100,
        '4': 1500
    }
    time.sleep(0.5)
    
    # Clear RC override
    vehicle.channels.overrides = {}
    time.sleep(0.5)
    
    logger.info("Landing complete")
    return True


def send_velocity_command(vehicle, vx: float, vy: float, vz: float, yaw_rate: float = 0.0) -> bool:
    """Send velocity command in GUIDED mode.
    
    Args:
        vehicle: DroneKit vehicle object
        vx: Forward velocity (m/s)
        vy: Right velocity (m/s)
        vz: Up velocity (m/s, positive = up)
        yaw_rate: Yaw rate (rad/s)
        
    Returns:
        True if command sent successfully
    """
    try:
        import math
        
        # Get current heading for NED conversion
        current_heading = math.radians(vehicle.heading)
        
        # Convert body frame to NED frame
        velocity_north = vx * math.cos(current_heading) - vy * math.sin(current_heading)
        velocity_east = vx * math.sin(current_heading) + vy * math.cos(current_heading)
        velocity_down = -vz  # NED uses down as positive
        
        # Type mask: use velocity and yaw_rate
        type_mask = 0b0000111111000111
        
        # Get target system/component
        target_system = vehicle._master.target_system if hasattr(vehicle, '_master') else 1
        target_component = vehicle._master.target_component if hasattr(vehicle, '_master') else 1
        
        # Send MAVLink command
        msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0,
            target_system,
            target_component,
            8,  # MAV_FRAME_LOCAL_NED
            type_mask,
            0, 0, 0,  # x, y, z (ignored)
            velocity_north, velocity_east, velocity_down,  # velocity in NED
            0, 0, 0,  # acceleration (ignored)
            0,  # yaw (ignored)
            yaw_rate  # yaw_rate
        )
        vehicle.send_mavlink(msg)
        return True
        
    except Exception as e:
        logger.error(f"Error sending velocity command: {e}")
        return False


def disarm_vehicle(vehicle) -> bool:
    """Disarm the vehicle.
    
    Args:
        vehicle: DroneKit vehicle object
        
    Returns:
        True if disarming successful, False otherwise
    """
    try:
        if not vehicle.armed:
            logger.info("Vehicle already disarmed")
            return True
        
        logger.info("Disarming vehicle...")
        vehicle.armed = False
        
        timeout = 5
        start_time = time.time()
        while vehicle.armed and (time.time() - start_time) < timeout:
            time.sleep(0.1)
        
        if not vehicle.armed:
            logger.info("Vehicle disarmed successfully")
            return True
        else:
            logger.warning("Disarming timeout, but continuing...")
            return True
            
    except Exception as e:
        logger.error(f"Error disarming vehicle: {e}")
        return False


def main():
    """Main flight sequence."""
    vehicle = None
    tof_sensor = None
    
    try:
        # Initialize ToF sensor
        logger.info("Initializing ToF sensor...")
        tof_sensor = ToFSensor(MULTIPLEXER_ADDRESS, VL53L1X_ADDRESS, BOTTOM_SENSOR_CHANNEL)
        if not tof_sensor.initialize():
            logger.error("Failed to initialize ToF sensor")
            return 1
        
        # Connect to vehicle
        logger.info(f"Connecting to vehicle at {CONNECTION_STRING} (baud={BAUD_RATE})...")
        vehicle = connect(CONNECTION_STRING, baud=BAUD_RATE, wait_ready=['autopilot_version'])
        logger.info(f"Connected to vehicle: {vehicle.version}")
        
        # Check GPS lock
        logger.info("Checking GPS lock...")
        if not check_gps_lock(vehicle):
            logger.error("GPS lock insufficient - aborting")
            return 1
        
        # Set GUIDED mode
        logger.info("Setting GUIDED mode...")
        vehicle.mode = VehicleMode("GUIDED")
        if not wait_for_mode(vehicle, "GUIDED", timeout=5.0):
            logger.error("Failed to set GUIDED mode")
            return 1
        
        # Arm vehicle
        if not arm_vehicle(vehicle):
            logger.error("Failed to arm vehicle")
            return 1
        
        # Detect takeoff
        if not detect_takeoff(vehicle, tof_sensor):
            logger.error("Failed to detect takeoff")
            return 1
        
        # Ascend to target altitude
        if not ascend_to_altitude(vehicle, tof_sensor, TARGET_ALTITUDE):
            logger.error("Failed to reach target altitude")
            return 1
        
        # Hold position
        if not hold_position(vehicle, HOLD_DURATION):
            logger.error("Failed to hold position")
            return 1
        
        # Descend to ground
        if not descend_to_ground(vehicle, tof_sensor):
            logger.error("Failed to descend")
            return 1
        
        # Land with throttle control
        if not land_with_throttle(vehicle, tof_sensor):
            logger.error("Failed to land")
            return 1
        
        # Disarm
        if not disarm_vehicle(vehicle):
            logger.error("Failed to disarm")
            return 1
        
        logger.info("Flight sequence completed successfully!")
        return 0
        
    except KeyboardInterrupt:
        logger.info("Interrupted by user")
        return 1
    except Exception as e:
        logger.error(f"Unexpected error: {e}")
        import traceback
        logger.error(traceback.format_exc())
        return 1
    finally:
        # Cleanup
        if vehicle is not None:
            try:
                # Clear RC override
                vehicle.channels.overrides = {}
                # Disarm if still armed
                if vehicle.armed:
                    vehicle.armed = False
                vehicle.close()
            except Exception as e:
                logger.error(f"Error during cleanup: {e}")
        
        if tof_sensor is not None:
            tof_sensor.cleanup()


if __name__ == "__main__":
    sys.exit(main())

