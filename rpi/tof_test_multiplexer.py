"""
Test script for TOF sensors connected via TCA9548A I2C multiplexer.
Tests two VL53L1X sensors on different multiplexer channels.
Includes retry logic for I2C EAGAIN errors during motor operation.
"""

import time
import errno
import board
import busio
import adafruit_vl53l1x
from smbus2 import SMBus

# Multiplexer configuration
MULTIPLEXER_ADDRESS = 0x70  # TCA9548A default address
CHANNEL_0 = 0  # First sensor (e.g., forward)
CHANNEL_1 = 1  # Second sensor (e.g., down)

# VL53L1X sensor address (default, can't be changed on Chinese sensors)
VL53L1X_ADDRESS = 0x29

# I2C retry configuration
MAX_I2C_RETRIES = 5
BASE_RETRY_DELAY = 0.02  # 20ms base delay


def select_multiplexer_channel(bus, channel, retry_count=0):
    """
    Select a channel on the TCA9548A multiplexer with retry logic for EAGAIN errors.
    
    Args:
        bus: SMBus instance
        channel: Channel number (0-7)
        retry_count: Current retry attempt (internal use)
    
    Raises:
        OSError: If channel selection fails after all retries
    """
    try:
        # TCA9548A channel selection: write bit mask (1 << channel)
        channel_mask = 1 << channel
        bus.write_byte(MULTIPLEXER_ADDRESS, channel_mask)
        time.sleep(0.01)  # Small delay for multiplexer switching
    except OSError as e:
        # Check if it's EAGAIN (errno 11) - Resource temporarily unavailable
        if (e.errno == errno.EAGAIN or e.errno == 11) and retry_count < MAX_I2C_RETRIES:
            # Exponential backoff: 20ms, 40ms, 80ms, 160ms, 320ms
            delay = BASE_RETRY_DELAY * (2 ** retry_count)
            print(f"  [Channel {channel}] I2C EAGAIN, retrying in {delay*1000:.0f}ms...")
            time.sleep(delay)
            return select_multiplexer_channel(bus, channel, retry_count + 1)
        else:
            raise


def test_sensors():
    """Test both TOF sensors via multiplexer."""
    
    # Initialize I2C bus using smbus2 for multiplexer control
    smbus = SMBus(1)
    
    # Initialize I2C bus using busio for adafruit library
    # Use 100kHz instead of default 400kHz for better EMI tolerance during motor operation
    i2c = busio.I2C(board.SCL, board.SDA, frequency=100000)
    print("I2C bus initialized at 100kHz for better EMI tolerance")
    
    sensors = {}
    
    print("Initializing TOF sensors via multiplexer...")
    print(f"Multiplexer address: 0x{MULTIPLEXER_ADDRESS:02X}")
    print(f"VL53L1X address: 0x{VL53L1X_ADDRESS:02X}\n")
    
    # Initialize sensor on channel 0
    try:
        print(f"Initializing sensor on channel {CHANNEL_0}...")
        select_multiplexer_channel(smbus, CHANNEL_0)
        time.sleep(0.1)  # Give sensor time to respond
        
        sensor_0 = adafruit_vl53l1x.VL53L1X(i2c, address=VL53L1X_ADDRESS)
        sensor_0.start_ranging()
        sensors[CHANNEL_0] = sensor_0
        print(f"âœ“ Sensor on channel {CHANNEL_0} initialized successfully")
    except Exception as e:
        print(f"âœ— Failed to initialize sensor on channel {CHANNEL_0}: {e}")
        sensors[CHANNEL_0] = None
    
    # Initialize sensor on channel 1
    try:
        print(f"Initializing sensor on channel {CHANNEL_1}...")
        select_multiplexer_channel(smbus, CHANNEL_1)
        time.sleep(0.1)  # Give sensor time to respond
        
        sensor_1 = adafruit_vl53l1x.VL53L1X(i2c, address=VL53L1X_ADDRESS)
        sensor_1.start_ranging()
        sensors[CHANNEL_1] = sensor_1
        print(f"âœ“ Sensor on channel {CHANNEL_1} initialized successfully")
    except Exception as e:
        print(f"âœ— Failed to initialize sensor on channel {CHANNEL_1}: {e}")
        sensors[CHANNEL_1] = None
    
    if sensors[CHANNEL_0] is None and sensors[CHANNEL_1] is None:
        print("\nâœ— No sensors initialized. Exiting.")
        smbus.close()
        return
    
    print("\n" + "="*50)
    print("Starting distance readings...")
    print("Press Ctrl+C to stop")
    print("="*50 + "\n")
    
    try:
        while True:
            # Read from sensor on channel 0
            if sensors[CHANNEL_0] is not None:
                try:
                    select_multiplexer_channel(smbus, CHANNEL_0)
                    time.sleep(0.02)  # Small delay after channel switch
                    
                    # Read with retry for EAGAIN errors
                    data_ready = False
                    for attempt in range(3):
                        try:
                            data_ready = sensor_0.data_ready
                            break
                        except OSError as e:
                            if e.errno == errno.EAGAIN or e.errno == 11:
                                time.sleep(0.01 * (attempt + 1))
                                continue
                            else:
                                raise
                    
                    if data_ready:
                        # Read distance with retry
                        distance_0 = None
                        for attempt in range(3):
                            try:
                                distance_0 = sensor_0.distance
                                break
                            except OSError as e:
                                if e.errno == errno.EAGAIN or e.errno == 11:
                                    time.sleep(0.01 * (attempt + 1))
                                    continue
                                else:
                                    raise
                        
                        if distance_0 is not None:
                            print(f"Channel {CHANNEL_0} (Forward): {distance_0:4.1f} cm", end="  |  ")
                            try:
                                sensor_0.clear_interrupt()
                            except OSError as e:
                                if e.errno != errno.EAGAIN and e.errno != 11:
                                    print(f"Clear interrupt error: {e}", end="  |  ")
                        else:
                            print(f"Channel {CHANNEL_0} (Forward): Read failed", end="  |  ")
                    else:
                        print(f"Channel {CHANNEL_0} (Forward): Waiting...", end="  |  ")
                except OSError as e:
                    if e.errno == errno.EAGAIN or e.errno == 11:
                        print(f"Channel {CHANNEL_0} (Forward): I2C busy (EAGAIN)", end="  |  ")
                    else:
                        print(f"Channel {CHANNEL_0} (Forward): Error - {e}", end="  |  ")
                except Exception as e:
                    print(f"Channel {CHANNEL_0} (Forward): Error - {e}", end="  |  ")
            else:
                print(f"Channel {CHANNEL_0} (Forward): Not initialized", end="  |  ")
            
            # Read from sensor on channel 1
            if sensors[CHANNEL_1] is not None:
                try:
                    select_multiplexer_channel(smbus, CHANNEL_1)
                    time.sleep(0.02)  # Small delay after channel switch
                    
                    # Read with retry for EAGAIN errors
                    data_ready = False
                    for attempt in range(3):
                        try:
                            data_ready = sensor_1.data_ready
                            break
                        except OSError as e:
                            if e.errno == errno.EAGAIN or e.errno == 11:
                                time.sleep(0.01 * (attempt + 1))
                                continue
                            else:
                                raise
                    
                    if data_ready:
                        # Read distance with retry
                        distance_1 = None
                        for attempt in range(3):
                            try:
                                distance_1 = sensor_1.distance
                                break
                            except OSError as e:
                                if e.errno == errno.EAGAIN or e.errno == 11:
                                    time.sleep(0.01 * (attempt + 1))
                                    continue
                                else:
                                    raise
                        
                        if distance_1 is not None:
                            print(f"Channel {CHANNEL_1} (Down): {distance_1:4.1f} cm")
                            try:
                                sensor_1.clear_interrupt()
                            except OSError as e:
                                if e.errno != errno.EAGAIN and e.errno != 11:
                                    print(f" (Clear interrupt error: {e})")
                        else:
                            print(f"Channel {CHANNEL_1} (Down): Read failed")
                    else:
                        print(f"Channel {CHANNEL_1} (Down): Waiting...")
                except OSError as e:
                    if e.errno == errno.EAGAIN or e.errno == 11:
                        print(f"Channel {CHANNEL_1} (Down): I2C busy (EAGAIN)")
                    else:
                        print(f"Channel {CHANNEL_1} (Down): Error - {e}")
                except Exception as e:
                    print(f"Channel {CHANNEL_1} (Down): Error - {e}")
            else:
                print(f"Channel {CHANNEL_1} (Down): Not initialized")
            
            time.sleep(0.1)  # 10 Hz reading rate
            
    except KeyboardInterrupt:
        print("\n\nStopping sensors...")
        
        # Stop ranging on both sensors
        if sensors[CHANNEL_0] is not None:
            try:
                select_multiplexer_channel(smbus, CHANNEL_0)
                time.sleep(0.01)
                sensors[CHANNEL_0].stop_ranging()
            except:
                pass
        
        if sensors[CHANNEL_1] is not None:
            try:
                select_multiplexer_channel(smbus, CHANNEL_1)
                time.sleep(0.01)
                sensors[CHANNEL_1].stop_ranging()
            except:
                pass
        
        print("Test completed!")
    
    finally:
        # Cleanup
        smbus.close()


if __name__ == "__main__":
    test_sensors()

