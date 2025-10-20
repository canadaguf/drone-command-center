import lgpio
import time
from smbus2 import SMBus

# GPIO and I2C config
XSHUT_PIN = 22
DEFAULT_ADDR = 0x29
NEW_ADDR = 0x30

# Open GPIO chip and I2C bus
chip = lgpio.gpiochip_open(0)
lgpio.gpio_claim_output(chip, XSHUT_PIN)
bus = SMBus(1)

# 1. Hold sensor in reset
lgpio.gpio_write(chip, XSHUT_PIN, 0)
time.sleep(0.01)

# 2. Release reset (sensor boots with address 0x29)
lgpio.gpio_write(chip, XSHUT_PIN, 1)
time.sleep(0.01)

# 3. Send command to change I2C address
# VL53L1X_I2C_SLAVE__DEVICE_ADDRESS = 0x0001 (16-bit register)
# We write the new 7-bit address shifted left by 1 (I2C convention)
bus.write_word_data(DEFAULT_ADDR, 0x0001, (NEW_ADDR << 1) & 0xFF)

print(f"Address changed from 0x{DEFAULT_ADDR:02X} to 0x{NEW_ADDR:02X}")

# Cleanup
lgpio.gpiochip_close(chip)
bus.close()
