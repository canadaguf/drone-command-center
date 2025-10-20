import time
import lgpio
from smbus2 import SMBus

DOWN_XSHUT = 22
FRONT_XSHUT = 23
ADDRESSES = {DOWN_XSHUT: 0x30, FRONT_XSHUT: 0x31}
DEFAULT_ADDR = 0x29

def set_i2c_address_on_boot(bus, old_addr, new_addr):
    """MUST be the first I2C transaction after power-up."""
    # Write to register 0x0001: [low_byte, high_byte]
    # Value = (new_addr << 1) as 16-bit, little-endian > [new_addr<<1, 0x00]
    bus.write_i2c_block_data(old_addr, 0x01, [(new_addr << 1) & 0xFF, 0x00])

chip = lgpio.gpiochip_open(0)
bus = SMBus(1)

# 1. Hold both in reset
for pin in [DOWN_XSHUT, FRONT_XSHUT]:
    lgpio.gpio_claim_output(chip, pin)
    lgpio.gpio_write(chip, pin, 0)
time.sleep(0.1)

# 2. Configure Down sensor
print("Configuring Down sensor...")
lgpio.gpio_write(chip, DOWN_XSHUT, 1)
time.sleep(0.01)  # Wait just enough for power-on (1-10 ms)
set_i2c_address_on_boot(bus, DEFAULT_ADDR, ADDRESSES[DOWN_XSHUT])
time.sleep(0.01)
print("  > Down set to 0x30")

# 3. Configure Front sensor (Down is now at 0x30, so Front can safely use 0x29)
print("Configuring Front sensor...")
lgpio.gpio_write(chip, FRONT_XSHUT, 1)
time.sleep(0.01)
set_i2c_address_on_boot(bus, DEFAULT_ADDR, ADDRESSES[FRONT_XSHUT])
time.sleep(0.01)
print("  > Front set to 0x31")

print("? Done.")

# In your main loop:
lgpio.gpio_write(chip, DOWN_XSHUT, 1)
lgpio.gpio_write(chip, FRONT_XSHUT, 0)
time.sleep(0.05)
read_down_distance()  # at 0x29

lgpio.gpio_write(chip, DOWN_XSHUT, 0)
lgpio.gpio_write(chip, FRONT_XSHUT, 1)
time.sleep(0.05)
read_front_distance()  # at 0x29

# Verify
print("\nScanning I2C bus:")
for addr in [0x29, 0x30, 0x31]:
    try:
        bus.read_byte(addr)
        print(f"  Found at 0x{addr:02X}")
    except OSError:
        print(f"  No device at 0x{addr:02X}")

lgpio.gpiochip_close(chip)
