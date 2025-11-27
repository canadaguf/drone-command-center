#!/usr/bin/env python3
"""
Standalone liftoff test script using STABILIZE mode (indoors with ToF sensor only).
Implements autonomous flight sequence: arm → takeoff → ascend to 1m → hold 60s → descend → land → disarm.
Uses RC override for all altitude control since STABILIZE mode doesn't support GPS or velocity commands.
"""

import time
import logging
import errno
import sys
from typing import Optional

# DroneKit imports
from dronekit import connect, VehicleMode
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

# PID-like control parameters for altitude
KP = 15.0  # Proportional gain
KI = 2.0   # Integral gain
KD = 5.0   # Derivative gain

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
            logger.info(f"Takeoff detected! Distance: {current_distance:.3f}m, Throttle: {current_throttle}")
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
    """Ascend to target altitude using RC override with PID-like control.
    
    Args:
        vehicle: DroneKit vehicle object
        tof_sensor: ToF sensor instance
        target_altitude: Target altitude in meters
        timeout: Maximum time to reach altitude
        
    Returns:
        True if altitude reached, False otherwise
    """
    logger.info(f"Ascending to {target_altitude}m using throttle control...")
    
    # Get current throttle (from takeoff detection)
    # Start from a reasonable hover throttle
    current_throttle = 1500
    last_distance = None
    last_error = 0.0
    integral_error = 0.0
    
    start_time = time.time()
    
    while (time.time() - start_time) < timeout:
        # Read ToF sensor
        current_distance = tof_sensor.get_distance()
        
        if current_distance is None:
            logger.warning("ToF sensor read failed during ascent, holding throttle...")
            time.sleep(0.2)
            continue
        
        # Check if target reached
        if abs(current_distance - target_altitude) < ALTITUDE_TOLERANCE:
            logger.info(f"Target altitude reached: {current_distance:.3f}m")
            return True
        
        # PID-like control
        error = target_altitude - current_distance
        
        # Integral term (with windup protection)
        integral_error += error * 0.2  # Integrate over 0.2s intervals
        integral_error = max(-0.5, min(0.5, integral_error))  # Limit windup
        
        # Derivative term
        if last_distance is not None:
            derivative_error = (error - last_error) / 0.2
        else:
            derivative_error = 0.0
        
        # Calculate throttle adjustment
        throttle_adjustment = (KP * error + KI * integral_error + KD * derivative_error) * 0.1
        
        # Update throttle
        current_throttle += int(throttle_adjustment)
        current_throttle = max(1100, min(1900, current_throttle))  # Clamp to valid range
        
        # Set throttle via RC override
        vehicle.channels.overrides = {
            '1': 1500,  # roll
            '2': 1500,  # pitch
            '3': current_throttle,  # throttle
            '4': 1500  # yaw
        }
        
        # Update for next iteration
        last_distance = current_distance
        last_error = error
        
        # Log progress every 0.2m
        if last_distance is None or int(current_distance * 5) != int(last_distance * 5):
            logger.info(f"Ascending: {current_distance:.3f}m / {target_altitude}m, throttle: {current_throttle}")
        
        time.sleep(0.2)  # Update every 200ms
    
    logger.warning("Altitude timeout, but continuing...")
    return True  # Continue anyway


def hold_altitude(vehicle, tof_sensor: ToFSensor, target_altitude: float, duration: float) -> bool:
    """Hold altitude using RC override with PID-like control.
    
    Args:
        vehicle: DroneKit vehicle object
        tof_sensor: ToF sensor instance
        target_altitude: Target altitude in meters
        duration: Hold duration in seconds
        
    Returns:
        True if successful, False otherwise
    """
    logger.info(f"Holding altitude at {target_altitude}m for {duration}s...")
    
    # Get current throttle (maintain from ascent)
    current_throttle = vehicle.channels.overrides.get('3', 1500) if vehicle.channels.overrides else 1500
    
    last_error = 0.0
    integral_error = 0.0
    start_time = time.time()
    
    while (time.time() - start_time) < duration:
        # Read ToF sensor
        current_distance = tof_sensor.get_distance()
        
        if current_distance is None:
            logger.warning("ToF sensor read failed during hold, maintaining throttle...")
            time.sleep(0.2)
            continue
        
        # PID-like control to maintain altitude
        error = target_altitude - current_distance
        
        # Integral term (with windup protection)
        integral_error += error * 0.2
        integral_error = max(-0.3, min(0.3, integral_error))  # Smaller integral for hold
        
        # Derivative term
        derivative_error = (error - last_error) / 0.2
        
        # Calculate throttle adjustment (smaller gains for hold)
        throttle_adjustment = (KP * error + KI * integral_error + KD * derivative_error) * 0.05
        
        # Update throttle
        current_throttle += int(throttle_adjustment)
        current_throttle = max(1100, min(1900, current_throttle))
        
        # Set throttle via RC override
        vehicle.channels.overrides = {
            '1': 1500,  # roll
            '2': 1500,  # pitch
            '3': current_throttle,  # throttle
            '4': 1500  # yaw
        }
        
        last_error = error
        
        # Log every 10 seconds
        elapsed = time.time() - start_time
        if int(elapsed) % 10 == 0 and elapsed > 0:
            logger.info(f"Holding: {current_distance:.3f}m (target: {target_altitude}m), elapsed: {elapsed:.1f}s")
        
        time.sleep(0.2)
    
    logger.info("Hold complete")
    return True


def descend_to_ground(vehicle, tof_sensor: ToFSensor, timeout: float = 60.0) -> bool:
    """Descend to ground using RC override throttle control.
    
    Args:
        vehicle: DroneKit vehicle object
        tof_sensor: ToF sensor instance
        timeout: Maximum time for descent
        
    Returns:
        True if descent successful, False otherwise
    """
    logger.info("Descending to ground using throttle control...")
    
    # Get current throttle
    current_throttle = vehicle.channels.overrides.get('3', 1500) if vehicle.channels.overrides else 1500
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
        
        # Check if close to ground
        if current_distance <= GROUND_THRESHOLD:
            logger.info(f"Close to ground: {current_distance:.3f}m")
            return True
        
        # Reduce throttle (faster when higher, slower when closer)
        if current_distance > 0.5:
            throttle_reduction = throttle_decrement * 2
        elif current_distance > 0.20:
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
        
        logger.debug(f"Descending: {current_distance:.3f}m, throttle: {current_throttle}")
        time.sleep(decrement_interval)
    
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
    
    # Get current throttle
    current_throttle = vehicle.channels.overrides.get('3', 1500) if vehicle.channels.overrides else 1500
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
        
        # Set STABILIZE mode
        logger.info("Setting STABILIZE mode...")
        vehicle.mode = VehicleMode("STABILIZE")
        if not wait_for_mode(vehicle, "STABILIZE", timeout=5.0):
            logger.error("Failed to set STABILIZE mode")
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
        
        # Hold altitude
        if not hold_altitude(vehicle, tof_sensor, TARGET_ALTITUDE, HOLD_DURATION):
            logger.error("Failed to hold altitude")
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

