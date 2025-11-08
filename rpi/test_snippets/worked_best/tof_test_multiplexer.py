"""
Test script for TOF sensors connected via TCA9548A I2C multiplexer.
Tests two VL53L1X sensors on different multiplexer channels.
"""

import time
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


def select_multiplexer_channel(bus, channel):
    """
    Select a channel on the TCA9548A multiplexer.
    
    Args:
        bus: SMBus instance
        channel: Channel number (0-7)
    """
    # TCA9548A channel selection: write bit mask (1 << channel)
    channel_mask = 1 << channel
    bus.write_byte(MULTIPLEXER_ADDRESS, channel_mask)
    time.sleep(0.01)  # Small delay for multiplexer switching


def test_sensors():
    """Test both TOF sensors via multiplexer."""
    
    # Initialize I2C bus using smbus2 for multiplexer control
    smbus = SMBus(1)
    
    # Initialize I2C bus using busio for adafruit library
    i2c = busio.I2C(board.SCL, board.SDA)
    
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
        print(f"✓ Sensor on channel {CHANNEL_0} initialized successfully")
    except Exception as e:
        print(f"✗ Failed to initialize sensor on channel {CHANNEL_0}: {e}")
        sensors[CHANNEL_0] = None
    
    # Initialize sensor on channel 1
    try:
        print(f"Initializing sensor on channel {CHANNEL_1}...")
        select_multiplexer_channel(smbus, CHANNEL_1)
        time.sleep(0.1)  # Give sensor time to respond
        
        sensor_1 = adafruit_vl53l1x.VL53L1X(i2c, address=VL53L1X_ADDRESS)
        sensor_1.start_ranging()
        sensors[CHANNEL_1] = sensor_1
        print(f"✓ Sensor on channel {CHANNEL_1} initialized successfully")
    except Exception as e:
        print(f"✗ Failed to initialize sensor on channel {CHANNEL_1}: {e}")
        sensors[CHANNEL_1] = None
    
    if sensors[CHANNEL_0] is None and sensors[CHANNEL_1] is None:
        print("\n✗ No sensors initialized. Exiting.")
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
                    
                    if sensor_0.data_ready:
                        distance_0 = sensor_0.distance
                        print(f"Channel {CHANNEL_0} (Forward): {distance_0:4.1f} cm", end="  |  ")
                        sensor_0.clear_interrupt()
                    else:
                        print(f"Channel {CHANNEL_0} (Forward): Waiting...", end="  |  ")
                except Exception as e:
                    print(f"Channel {CHANNEL_0} (Forward): Error - {e}", end="  |  ")
            else:
                print(f"Channel {CHANNEL_0} (Forward): Not initialized", end="  |  ")
            
            # Read from sensor on channel 1
            if sensors[CHANNEL_1] is not None:
                try:
                    select_multiplexer_channel(smbus, CHANNEL_1)
                    time.sleep(0.02)  # Small delay after channel switch
                    
                    if sensor_1.data_ready:
                        distance_1 = sensor_1.distance
                        print(f"Channel {CHANNEL_1} (Down): {distance_1:4.1f} cm")
                        sensor_1.clear_interrupt()
                    else:
                        print(f"Channel {CHANNEL_1} (Down): Waiting...")
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

