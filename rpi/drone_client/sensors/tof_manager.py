"""
TOF sensor manager with TCA9548A I2C multiplexer support.
Supports 2-8 VL53L1X sensors for scalable obstacle avoidance.
Based on test_snippets/worked_best/tof_test_multiplexer.py
"""

import time
import logging
import threading
from typing import Dict, Any, List, Optional, Tuple
import smbus2
import board
import busio
import adafruit_vl53l1x

logger = logging.getLogger(__name__)

class TOFManager:
    """TOF sensor manager with TCA9548A multiplexer support."""
    
    def __init__(self, config: Dict[str, Any]):
        """Initialize TOF manager.
        
        Args:
            config: TOF configuration dictionary
        """
        self.config = config
        self.multiplexer_address = config.get('multiplexer_address', 0x70)
        self.sensors_config = config.get('sensors', [])
        
        # I2C bus
        self.bus = None
        self.bus_lock = threading.Lock()
        
        # Sensor states
        self.sensors = {}
        self.last_readings = {}
        self.reading_thread = None
        self.running = False
        
        # Initialize sensors
        self._initialize_sensors()
    
    def _initialize_sensors(self) -> bool:
        """Initialize all configured sensors.
        
        Returns:
            True if initialization successful, False otherwise
        """
        try:
            # Initialize I2C bus for multiplexer control (smbus2)
            self.bus = smbus2.SMBus(1)  # Use I2C bus 1
            logger.info("I2C bus (smbus2) initialized for multiplexer control")
            
            # Initialize I2C bus for Adafruit library (busio)
            try:
                self.i2c_busio = busio.I2C(board.SCL, board.SDA)
                logger.info("I2C bus (busio) initialized for Adafruit library")
            except Exception as e:
                logger.error(f"Failed to initialize busio I2C: {e}")
                return False
            
            # Initialize each sensor
            for sensor_config in self.sensors_config:
                name = sensor_config['name']
                channel = sensor_config['channel']
                direction = sensor_config['direction']
                max_range = sensor_config.get('max_range', 4.0)
                
                sensor = TOFSensor(
                    name=name,
                    channel=channel,
                    direction=direction,
                    max_range=max_range,
                    smbus=self.bus,
                    i2c_busio=self.i2c_busio,
                    multiplexer_address=self.multiplexer_address
                )
                
                if sensor.initialize():
                    self.sensors[name] = sensor
                    self.last_readings[name] = {
                        'distance': None,
                        'timestamp': time.time(),
                        'valid': False
                    }
                    logger.info(f"Initialized TOF sensor: {name} (channel {channel}, {direction})")
                else:
                    logger.error(f"Failed to initialize TOF sensor: {name}")
            
            if not self.sensors:
                logger.error("No TOF sensors initialized")
                return False
            
            logger.info(f"Initialized {len(self.sensors)} TOF sensors")
            return True
            
        except Exception as e:
            logger.error(f"Failed to initialize TOF manager: {e}")
            import traceback
            logger.debug(traceback.format_exc())
            return False
    
    def start_reading(self) -> bool:
        """Start continuous reading thread.
        
        Returns:
            True if started successfully, False otherwise
        """
        if self.reading_thread and self.reading_thread.is_alive():
            logger.warning("Reading thread already running")
            return True
        
        try:
            self.running = True
            self.reading_thread = threading.Thread(target=self._reading_loop, daemon=True)
            self.reading_thread.start()
            logger.info("TOF reading thread started")
            return True
        except Exception as e:
            logger.error(f"Failed to start reading thread: {e}")
            return False
    
    def stop_reading(self) -> None:
        """Stop continuous reading thread."""
        self.running = False
        if self.reading_thread:
            self.reading_thread.join(timeout=2.0)
        logger.info("TOF reading thread stopped")
    
    def _reading_loop(self) -> None:
        """Continuous reading loop."""
        while self.running:
            try:
                with self.bus_lock:
                    for name, sensor in self.sensors.items():
                        try:
                            distance = sensor.read_distance()
                            if distance is not None:
                                self.last_readings[name] = {
                                    'distance': distance,
                                    'timestamp': time.time(),
                                    'valid': True
                                }
                            else:
                                # Mark as invalid but keep last valid reading
                                if name in self.last_readings:
                                    self.last_readings[name]['valid'] = False
                        except Exception as e:
                            logger.error(f"Error reading sensor {name}: {e}")
                            if name in self.last_readings:
                                self.last_readings[name]['valid'] = False
                
                time.sleep(0.1)  # 10 Hz reading rate
                
            except Exception as e:
                logger.error(f"Error in TOF reading loop: {e}")
                time.sleep(1.0)
    
    def get_distance(self, sensor_name: str) -> Optional[float]:
        """Get distance from specific sensor.
        
        Args:
            sensor_name: Name of sensor
            
        Returns:
            Distance in meters, None if not available
        """
        if sensor_name not in self.last_readings:
            return None
        
        reading = self.last_readings[sensor_name]
        if reading and reading['valid']:
            return reading['distance']
        
        return None
    
    def get_all_readings(self) -> Dict[str, Dict[str, Any]]:
        """Get all sensor readings.
        
        Returns:
            Dictionary of sensor readings
        """
        return self.last_readings.copy()
    
    def get_forward_distance(self) -> Optional[float]:
        """Get forward sensor distance.
        
        Returns:
            Forward distance in meters, None if not available
        """
        return self.get_distance('forward')
    
    def get_down_distance(self) -> Optional[float]:
        """Get downward sensor distance.
        
        Returns:
            Downward distance in meters, None if not available
        """
        return self.get_distance('down')
    
    def is_obstacle_detected(self, sensor_name: str, threshold: float = 2.0) -> bool:
        """Check if obstacle detected by sensor.
        
        Args:
            sensor_name: Name of sensor
            threshold: Distance threshold in meters
            
        Returns:
            True if obstacle detected, False otherwise
        """
        distance = self.get_distance(sensor_name)
        if distance is None:
            return False
        
        return distance < threshold
    
    def is_ground_detected(self, threshold: float = 1.0) -> bool:
        """Check if ground detected by downward sensor.
        
        Args:
            threshold: Distance threshold in meters
            
        Returns:
            True if ground detected, False otherwise
        """
        return self.is_obstacle_detected('down', threshold)
    
    def get_sensor_status(self) -> Dict[str, Any]:
        """Get status of all sensors.
        
        Returns:
            Dictionary containing sensor status
        """
        status = {
            'num_sensors': len(self.sensors),
            'sensor_names': list(self.sensors.keys()),
            'readings': {}
        }
        
        for name, reading in self.last_readings.items():
            if reading:
                status['readings'][name] = {
                    'distance': reading['distance'],
                    'valid': reading['valid'],
                    'age': time.time() - reading['timestamp'] if 'timestamp' in reading else None
                }
            else:
                status['readings'][name] = {
                    'distance': None,
                    'valid': False,
                    'age': None
                }
        
        return status
    
    def cleanup(self) -> None:
        """Cleanup resources."""
        self.stop_reading()
        
        # Stop ranging on all sensors
        for name, sensor in self.sensors.items():
            try:
                sensor.stop_ranging()
            except Exception as e:
                logger.warning(f"Error stopping sensor {name}: {e}")
        
        # Close I2C buses
        if self.bus:
            try:
                self.bus.close()
            except Exception as e:
                logger.warning(f"Error closing smbus: {e}")
        
        if hasattr(self, 'i2c_busio') and self.i2c_busio:
            try:
                self.i2c_busio.deinit()
            except Exception as e:
                logger.warning(f"Error closing busio I2C: {e}")
        
        logger.info("TOF manager cleaned up")

class TOFSensor:
    """Individual TOF sensor with multiplexer support."""
    
    def __init__(self, name: str, channel: int, direction: str, 
                 max_range: float, smbus: smbus2.SMBus, i2c_busio: busio.I2C,
                 multiplexer_address: int):
        """Initialize TOF sensor.
        
        Args:
            name: Sensor name
            channel: Multiplexer channel
            direction: Sensor direction ('front', 'down', etc.)
            max_range: Maximum range in meters
            smbus: SMBus instance for multiplexer control
            i2c_busio: Busio I2C instance for Adafruit library
            multiplexer_address: Multiplexer I2C address
        """
        self.name = name
        self.channel = channel
        self.direction = direction
        self.max_range = max_range
        self.smbus = smbus
        self.i2c_busio = i2c_busio
        self.multiplexer_address = multiplexer_address
        
        # VL53L1X parameters
        self.vl53l1x_address = 0x29
        self.adafruit_sensor = None
        self.initialized = False
    
    def initialize(self) -> bool:
        """Initialize sensor.
        
        Returns:
            True if initialization successful, False otherwise
        """
        try:
            # Select multiplexer channel
            self._select_channel()
            time.sleep(0.1)  # Give sensor time to respond after channel switch
            
            # Initialize VL53L1X sensor using Adafruit library
            try:
                self.adafruit_sensor = adafruit_vl53l1x.VL53L1X(
                    self.i2c_busio, 
                    address=self.vl53l1x_address
                )
                self.adafruit_sensor.start_ranging()
                logger.info(f"VL53L1X sensor {self.name} started ranging")
            except Exception as e:
                logger.error(f"Failed to initialize Adafruit VL53L1X on {self.name}: {e}")
                return False
            
            self.initialized = True
            logger.info(f"TOF sensor {self.name} initialized successfully")
            return True
            
        except Exception as e:
            logger.error(f"Failed to initialize sensor {self.name}: {e}")
            import traceback
            logger.debug(traceback.format_exc())
            return False
    
    def _select_channel(self) -> None:
        """Select multiplexer channel."""
        try:
            # TCA9548A channel selection (1 << channel)
            channel_mask = 1 << self.channel
            self.smbus.write_byte(self.multiplexer_address, channel_mask)
            time.sleep(0.01)  # Small delay for multiplexer switching
        except Exception as e:
            logger.error(f"Failed to select channel {self.channel}: {e}")
            raise
    
    def stop_ranging(self) -> None:
        """Stop ranging on sensor."""
        if self.adafruit_sensor:
            try:
                self._select_channel()
                time.sleep(0.01)
                self.adafruit_sensor.stop_ranging()
            except Exception as e:
                logger.warning(f"Error stopping ranging on {self.name}: {e}")
    
    def read_distance(self) -> Optional[float]:
        """Read distance from sensor.
        
        Returns:
            Distance in meters, None if reading failed
        """
        if not self.initialized or not self.adafruit_sensor:
            return None
        
        try:
            # Select multiplexer channel
            self._select_channel()
            time.sleep(0.02)  # Small delay after channel switch
            
            # Check if data is ready
            if not self.adafruit_sensor.data_ready:
                return None
            
            # Read distance (returns value in centimeters)
            distance_cm = self.adafruit_sensor.distance
            
            # Convert to meters
            distance_m = distance_cm / 100.0
            
            # Validate reading (sensor can return 0 or very large values when out of range)
            if distance_cm <= 0 or distance_cm > (self.max_range * 100):
                return None
            
            # Clear interrupt for next reading
            self.adafruit_sensor.clear_interrupt()
            
            return distance_m
            
        except Exception as e:
            logger.error(f"Failed to read distance from {self.name}: {e}")
            return None
    
    def get_info(self) -> Dict[str, Any]:
        """Get sensor information.
        
        Returns:
            Dictionary containing sensor info
        """
        return {
            'name': self.name,
            'channel': self.channel,
            'direction': self.direction,
            'max_range': self.max_range,
            'initialized': self.initialized
        }
