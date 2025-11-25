"""
ToF (Time-of-Flight) distance sensor manager.
Manages VL53L1X sensors connected via TCA9548A I2C multiplexer.
"""

import time
import errno
import logging
import board
import busio
import adafruit_vl53l1x
from smbus2 import SMBus
from typing import Dict, Any, Optional

logger = logging.getLogger(__name__)

# Multiplexer configuration
MULTIPLEXER_ADDRESS = 0x70  # TCA9548A default address
VL53L1X_ADDRESS = 0x29  # VL53L1X sensor address (default)

# I2C retry configuration
MAX_I2C_RETRIES = 5
BASE_RETRY_DELAY = 0.02  # 20ms base delay


class ToFSensorManager:
    """Manager for ToF sensors via I2C multiplexer."""
    
    def __init__(self, config: Dict[str, Any]):
        """Initialize ToF sensor manager.
        
        Args:
            config: ToF sensor configuration dictionary
        """
        self.config = config
        self.multiplexer_address = config.get('multiplexer_address', MULTIPLEXER_ADDRESS)
        
        # Parse sensor configuration
        sensors_config = config.get('sensors', [])
        self.sensors = {}
        self.channel_map = {}  # Maps sensor name to channel
        
        for sensor_config in sensors_config:
            name = sensor_config.get('name')
            channel = sensor_config.get('channel')
            if name and channel is not None:
                self.channel_map[name] = channel
        
        # I2C bus instances
        self.smbus = None
        self.i2c = None
        
        # Sensor instances
        self.sensor_instances = {}
        
        # Last readings
        self.last_readings = {
            'forward': None,
            'down': None
        }
        
        # Sensor state
        self.initialized = False
        
    def initialize(self) -> bool:
        """Initialize sensors.
        
        Returns:
            True if initialization successful, False otherwise
        """
        try:
            logger.info("Initializing ToF sensors...")
            
            # Initialize I2C bus using smbus2 for multiplexer control
            self.smbus = SMBus(1)
            
            # Initialize I2C bus using busio for adafruit library
            # Use 100kHz instead of default 400kHz for better EMI tolerance during motor operation
            self.i2c = busio.I2C(board.SCL, board.SDA, frequency=100000)
            logger.info("I2C bus initialized at 100kHz for better EMI tolerance")
            
            # Initialize sensors
            for name, channel in self.channel_map.items():
                try:
                    logger.info(f"Initializing sensor '{name}' on channel {channel}...")
                    self._select_channel(channel)
                    time.sleep(0.1)  # Give sensor time to respond
                    
                    sensor = adafruit_vl53l1x.VL53L1X(self.i2c, address=VL53L1X_ADDRESS)
                    sensor.start_ranging()
                    self.sensor_instances[name] = {
                        'sensor': sensor,
                        'channel': channel
                    }
                    logger.info(f"✓ Sensor '{name}' initialized successfully")
                except Exception as e:
                    logger.error(f"✗ Failed to initialize sensor '{name}': {e}")
                    self.sensor_instances[name] = None
            
            # Check if at least one sensor initialized
            if not any(inst is not None for inst in self.sensor_instances.values()):
                logger.error("No sensors initialized successfully")
                return False
            
            self.initialized = True
            logger.info("ToF sensor manager initialized successfully")
            return True
            
        except Exception as e:
            logger.error(f"Failed to initialize ToF sensors: {e}")
            import traceback
            logger.error(traceback.format_exc())
            return False
    
    def _select_channel(self, channel: int, retry_count: int = 0) -> None:
        """Select a channel on the TCA9548A multiplexer with retry logic.
        
        Args:
            channel: Channel number (0-7)
            retry_count: Current retry attempt (internal use)
        
        Raises:
            OSError: If channel selection fails after all retries
        """
        try:
            # TCA9548A channel selection: write bit mask (1 << channel)
            channel_mask = 1 << channel
            self.smbus.write_byte(self.multiplexer_address, channel_mask)
            time.sleep(0.01)  # Small delay for multiplexer switching
        except OSError as e:
            # Check if it's EAGAIN (errno 11) - Resource temporarily unavailable
            if (e.errno == errno.EAGAIN or e.errno == 11) and retry_count < MAX_I2C_RETRIES:
                # Exponential backoff: 20ms, 40ms, 80ms, 160ms, 320ms
                delay = BASE_RETRY_DELAY * (2 ** retry_count)
                logger.debug(f"  [Channel {channel}] I2C EAGAIN, retrying in {delay*1000:.0f}ms...")
                time.sleep(delay)
                return self._select_channel(channel, retry_count + 1)
            else:
                raise
    
    def _read_sensor(self, name: str) -> Optional[float]:
        """Read distance from a sensor.
        
        Args:
            name: Sensor name (e.g., 'forward', 'down')
            
        Returns:
            Distance in meters, or None if read failed
        """
        if name not in self.sensor_instances:
            return None
        
        sensor_info = self.sensor_instances.get(name)
        if sensor_info is None:
            return None
        
        sensor = sensor_info['sensor']
        channel = sensor_info['channel']
        
        try:
            # Select channel
            self._select_channel(channel)
            time.sleep(0.02)  # Small delay after channel switch
            
            # Check if data is ready with retry
            data_ready = False
            for attempt in range(3):
                try:
                    data_ready = sensor.data_ready
                    break
                except OSError as e:
                    if e.errno == errno.EAGAIN or e.errno == 11:
                        time.sleep(0.01 * (attempt + 1))
                        continue
                    else:
                        raise
            
            if not data_ready:
                return None
            
            # Read distance with retry
            distance_cm = None
            for attempt in range(3):
                try:
                    distance_cm = sensor.distance
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
                sensor.clear_interrupt()
            except OSError as e:
                if e.errno != errno.EAGAIN and e.errno != 11:
                    logger.debug(f"Clear interrupt error for {name}: {e}")
            
            # Convert cm to meters
            distance_m = distance_cm / 100.0
            
            # Validate reading (sensor range is typically 0.04m to 4m)
            if distance_m < 0.0 or distance_m > 4.0:
                logger.debug(f"Sensor {name}: Reading out of valid range ({distance_m:.3f}m)")
                return None
            
            return distance_m
            
        except OSError as e:
            if e.errno == errno.EAGAIN or e.errno == 11:
                logger.debug(f"Sensor {name}: I2C busy (EAGAIN)")
            else:
                logger.warning(f"Sensor {name}: Error - {e}")
            return None
        except Exception as e:
            logger.warning(f"Sensor {name}: Unexpected error - {e}")
            return None
    
    def get_forward_distance(self) -> Optional[float]:
        """Get forward sensor distance.
        
        Returns:
            Distance in meters, or None if unavailable
        """
        if not self.initialized:
            return None
        
        distance = self._read_sensor('forward')
        if distance is not None:
            self.last_readings['forward'] = distance
        return distance
    
    def get_bottom_distance(self) -> Optional[float]:
        """Get bottom sensor distance (height above ground).
        
        Returns:
            Distance in meters, or None if unavailable
        """
        if not self.initialized:
            return None
        
        distance = self._read_sensor('down')
        if distance is not None:
            self.last_readings['down'] = distance
        return distance
    
    def get_all_readings(self) -> Dict[str, Dict[str, Any]]:
        """Get all sensor readings with validation.
        
        Returns:
            Dictionary with sensor readings and validity flags
        """
        forward_dist = self.get_forward_distance()
        bottom_dist = self.get_bottom_distance()
        
        # Validate readings
        forward_valid = forward_dist is not None and 0.0 <= forward_dist <= 4.0
        bottom_valid = bottom_dist is not None and 0.0 <= bottom_dist <= 4.0
        
        return {
            'forward': {
                'distance': forward_dist if forward_valid else None,
                'valid': forward_valid
            },
            'down': {
                'distance': bottom_dist if bottom_valid else None,
                'valid': bottom_valid
            }
        }
    
    def cleanup(self) -> None:
        """Cleanup sensors and close connections."""
        logger.info("Cleaning up ToF sensors...")
        
        # Stop ranging on all sensors
        for name, sensor_info in self.sensor_instances.items():
            if sensor_info is not None:
                try:
                    sensor = sensor_info['sensor']
                    channel = sensor_info['channel']
                    self._select_channel(channel)
                    time.sleep(0.01)
                    sensor.stop_ranging()
                except Exception as e:
                    logger.debug(f"Error stopping sensor {name}: {e}")
        
        # Close I2C buses
        if self.smbus:
            try:
                self.smbus.close()
            except Exception:
                pass
        
        self.initialized = False
        logger.info("ToF sensors cleaned up")

