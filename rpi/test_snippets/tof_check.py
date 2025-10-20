# check_vl53l1x_id.py
import time
import lgpio
from smbus2 import SMBus

XSHUT = 22  # test one sensor at a time
chip = lgpio.gpiochip_open(0)
lgpio.gpio_claim_output(chip, XSHUT)

# Reset sensor
lgpio.gpio_write(chip, XSHUT, 0)
time.sleep(0.01)
lgpio.gpio_write(chip, XSHUT, 1)
time.sleep(0.05)  # Let it boot

bus = SMBus(1)
try:
    # Read 1 byte from register 0x010F (device ID)
    device_id = bus.read_byte_data(0x29, 0x010F)
    print(f"Device ID: 0x{device_id:02X} ({device_id})")
    
    if device_id == 0xEE:
        print("Genuine ST VL53L1X detected.")
    elif device_id == 0xAA:
        print("This is a VL53L0X (older model) - not VL53L1X!")
    else:
        print("Unknown or clone sensor (ID not 0xEE).")
except Exception as e:
    print("Error reading ID:", e)

lgpio.gpiochip_close(chip)
