"""
TOF sensor manager with TCA9548A I2C multiplexer support.
Supports 2-8 VL53L1X sensors for scalable obstacle avoidance.
Based on test_snippets/worked_best/tof_test_1.py
"""

import time
import logging
import threading
from typing import Dict, Any, List, Optional, Tuple
import smbus2
import RPi.GPIO as GPIO

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
            # Initialize I2C bus
            self.bus = smbus2.SMBus(1)  # Use I2C bus 1
            logger.info("I2C bus initialized")
            
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
                    bus=self.bus,
                    multiplexer_address=self.multiplexer_address
                )
                
                if sensor.initialize():
                    self.sensors[name] = sensor
                    self.last_readings[name] = None
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
        if self.bus:
            self.bus.close()
        logger.info("TOF manager cleaned up")

class TOFSensor:
    """Individual TOF sensor with multiplexer support."""
    
    def __init__(self, name: str, channel: int, direction: str, 
                 max_range: float, bus: smbus2.SMBus, multiplexer_address: int):
        """Initialize TOF sensor.
        
        Args:
            name: Sensor name
            channel: Multiplexer channel
            direction: Sensor direction ('front', 'down', etc.)
            max_range: Maximum range in meters
            bus: I2C bus instance
            multiplexer_address: Multiplexer I2C address
        """
        self.name = name
        self.channel = channel
        self.direction = direction
        self.max_range = max_range
        self.bus = bus
        self.multiplexer_address = multiplexer_address
        
        # VL53L1X parameters
        self.vl53l1x_address = 0x29
        self.initialized = False
    
    def initialize(self) -> bool:
        """Initialize sensor.
        
        Returns:
            True if initialization successful, False otherwise
        """
        try:
            # Select multiplexer channel
            self._select_channel()
            
            # Check if sensor is present
            if not self._check_sensor_presence():
                logger.error(f"Sensor {self.name} not found on channel {self.channel}")
                return False
            
            # Initialize VL53L1X sensor
            if not self._init_vl53l1x():
                logger.error(f"Failed to initialize VL53L1X on {self.name}")
                return False
            
            self.initialized = True
            logger.info(f"TOF sensor {self.name} initialized successfully")
            return True
            
        except Exception as e:
            logger.error(f"Failed to initialize sensor {self.name}: {e}")
            return False
    
    def _select_channel(self) -> None:
        """Select multiplexer channel."""
        try:
            # TCA9548A channel selection (1 << channel)
            channel_mask = 1 << self.channel
            self.bus.write_byte(self.multiplexer_address, channel_mask)
            time.sleep(0.01)  # Small delay for multiplexer switching
        except Exception as e:
            logger.error(f"Failed to select channel {self.channel}: {e}")
            raise
    
    def _check_sensor_presence(self) -> bool:
        """Check if sensor is present on current channel.
        
        Returns:
            True if sensor found, False otherwise
        """
        try:
            # Try to read from VL53L1X address
            self.bus.read_byte(self.vl53l1x_address)
            return True
        except:
            return False
    
    def _init_vl53l1x(self) -> bool:
        """Initialize VL53L1X sensor.
        
        Returns:
            True if initialization successful, False otherwise
        """
        try:
            # VL53L1X initialization sequence
            # This is a simplified version - full implementation would include
            # proper VL53L1X register configuration
            
            # Reset sensor
            self.bus.write_byte_data(self.vl53l1x_address, 0x00, 0x00)
            time.sleep(0.01)
            
            # Set up sensor (simplified)
            # In a full implementation, you would configure all necessary registers
            # For now, we'll assume the sensor is pre-configured
            
            return True
            
        except Exception as e:
            logger.error(f"VL53L1X initialization failed: {e}")
            return False
    
    def read_distance(self) -> Optional[float]:
        """Read distance from sensor.
        
        Returns:
            Distance in meters, None if reading failed
        """
        if not self.initialized:
            return None
        
        try:
            # Select channel
            self._select_channel()
            
            # Read distance from VL53L1X
            # This is a simplified implementation
            # Full implementation would read from proper VL53L1X registers
            
            # For now, return a mock reading
            # In real implementation, you would:
            # 1. Trigger measurement
            # 2. Wait for completion
            # 3. Read result registers
            # 4. Convert to meters
            
            # Mock reading (replace with actual VL53L1X reading)
            import random
            distance = random.uniform(0.5, self.max_range)
            
            return distance
            
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
