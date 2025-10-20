import board
import busio
import digitalio
import adafruit_vl53l1x

# Optional: control XSHUT via GPIO if needed
# But for reading, just use I2C
'''
i2c = busio.I2C(board.SCL, board.SDA)
tof = adafruit_vl53l1x.VL53L1X(i2c, address=0x29)  # < custom address!

tof.start_ranging()
print("Distance:", tof.distance, "cm")
tof.stop_ranging()
'''
# In your main loop:
lgpio.gpio_write(chip, DOWN_XSHUT, 1)
lgpio.gpio_write(chip, FRONT_XSHUT, 0)
time.sleep(0.05)
read_down_distance()  # at 0x29

lgpio.gpio_write(chip, DOWN_XSHUT, 0)
lgpio.gpio_write(chip, FRONT_XSHUT, 1)
time.sleep(0.05)
read_front_distance()  # at 0x29
