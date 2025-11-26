# ToF 400 Sensors + PCA9548A + Raspberry Pi Wiring Diagram

## Overview
This guide shows how to wire two ToF 400 sensors (VL53L0X or similar) to a Raspberry Pi using a PCA9548A I2C multiplexer to overcome identical I2C addresses.

## Components Needed
- Raspberry Pi (any model with I2C pins)
- PCA9548A I2C Multiplexer Board
- 2x ToF 400 Sensors
- Jumper wires
- Breadboard (optional)

---

## Wiring Diagram

```
                    RASPBERRY PI
                    ┌─────────────┐
                    │             │
                    │  Pin 1  ●   │ 3.3V ──────┐
                    │  Pin 2  ●   │            │
                    │  Pin 3  ●   │ SDA ───────┼──────┐
                    │  Pin 4  ●   │            │      │
                    │  Pin 5  ●   │ SCL ───────┼────┐ │
                    │  Pin 6  ●   │ GND ───────┼──┐ │ │
                    │         ...  │            │  │ │ │
                    └─────────────┘            │  │ │ │
                                               │  │ │ │
                                               │  │ │ │
                    PCA9548A                   │  │ │ │
            ┌──────────────────────────┐       │  │ │ │
            │                          │       │  │ │ │
            │  VIN  ●─────────────────────────┘  │ │ │
            │  GND  ●───────────────────────────┘ │ │
            │  SCL  ●─────────────────────────────┘ │
            │  SDA  ●───────────────────────────────┘
            │                          │
            │  SD0  ●──────────┐       │
            │  SC0  ●────────┐ │       │
            │                │ │       │
            │  SD1  ●──────┐ │ │       │
            │  SC1  ●────┐ │ │ │       │
            │            │ │ │ │       │
            │  SD2-SD7   │ │ │ │       │
            │  SC2-SC7   │ │ │ │       │
            │  (unused)  │ │ │ │       │
            └────────────┼─┼─┼─┼───────┘
                         │ │ │ │
                         │ │ │ │
        ┌────────────────┘ │ │ └────────────────┐
        │ ┌────────────────┘ └────────────────┐ │
        │ │                                   │ │
        │ │    ToF Sensor #1                 │ │    ToF Sensor #2
        │ │    ┌──────────┐                  │ │    ┌──────────┐
        │ │    │          │                  │ │    │          │
        │ │    │  VCC  ●──┼──────────────────┼─┼────┼──●  VCC  │
        │ │    │  GND  ●──┼──────────────────┼─┼────┼──●  GND  │
        │ └────┼──●  SDA  │                  │ └────┼──●  SDA  │
        └──────┼──●  SCL  │                  └──────┼──●  SCL  │
               │          │                         │          │
               └──────────┘                         └──────────┘
```

---

## Pin Connection Table

### Raspberry Pi Connections

| Raspberry Pi Pin | Function | Connect To |
|-----------------|----------|------------|
| Pin 1 (3.3V)    | Power    | PCA9548A VIN |
| Pin 3 (GPIO 2)  | SDA      | PCA9548A SDA |
| Pin 5 (GPIO 3)  | SCL      | PCA9548A SCL |
| Pin 6 (GND)     | Ground   | PCA9548A GND |

**Alternative:** If your sensors require 5V, use Pin 2 (5V) instead of Pin 1 (3.3V). Most ToF sensors work with 3.3V.

### PCA9548A Connections

| PCA9548A Pin | Connect To |
|-------------|------------|
| VIN         | Raspberry Pi 3.3V (Pin 1) |
| GND         | Raspberry Pi GND (Pin 6) |
| SDA         | Raspberry Pi SDA (Pin 3) |
| SCL         | Raspberry Pi SCL (Pin 5) |
| SD0         | ToF Sensor #1 SDA |
| SC0         | ToF Sensor #1 SCL |
| SD1         | ToF Sensor #2 SDA |
| SC1         | ToF Sensor #2 SCL |

### ToF Sensor #1 Connections

| ToF Sensor #1 Pin | Connect To |
|------------------|------------|
| VCC              | PCA9548A VIN (or direct to Pi 3.3V) |
| GND              | PCA9548A GND (or direct to Pi GND) |
| SDA              | PCA9548A SD0 (Channel 0) |
| SCL              | PCA9548A SC0 (Channel 0) |

### ToF Sensor #2 Connections

| ToF Sensor #2 Pin | Connect To |
|------------------|------------|
| VCC              | PCA9548A VIN (or direct to Pi 3.3V) |
| GND              | PCA9548A GND (or direct to Pi GND) |
| SDA              | PCA9548A SD1 (Channel 1) |
| SCL              | PCA9548A SC1 (Channel 1) |

---

## Step-by-Step Wiring Instructions

1. **Power Down**: Ensure your Raspberry Pi is powered off before making connections.

2. **Connect PCA9548A to Raspberry Pi**:
   - VIN → Pin 1 (3.3V)
   - GND → Pin 6 (GND)
   - SDA → Pin 3 (GPIO 2 / SDA)
   - SCL → Pin 5 (GPIO 3 / SCL)

3. **Connect ToF Sensor #1 to PCA9548A Channel 0**:
   - Sensor VCC → PCA9548A VIN (or Pi 3.3V)
   - Sensor GND → PCA9548A GND (or Pi GND)
   - Sensor SDA → PCA9548A SD0
   - Sensor SCL → PCA9548A SC0

4. **Connect ToF Sensor #2 to PCA9548A Channel 1**:
   - Sensor VCC → PCA9548A VIN (or Pi 3.3V)
   - Sensor GND → PCA9548A GND (or Pi GND)
   - Sensor SDA → PCA9548A SD1
   - Sensor SCL → PCA9548A SC1

---

## Enable I2C on Raspberry Pi

Before using the sensors, enable I2C:

```bash
sudo raspi-config
# Navigate to: Interface Options → I2C → Enable
```

Or edit directly:

```bash
sudo nano /boot/config.txt
# Add or uncomment:
dtparam=i2c_arm=on
```

Reboot:
```bash
sudo reboot
```

Install I2C tools:
```bash
sudo apt-get update
sudo apt-get install i2c-tools python3-smbus
```

---

## Testing the Setup

1. **Detect I2C devices**:
```bash
sudo i2cdetect -y 1
```

You should see the PCA9548A at address `0x70` (default).

2. **Test accessing sensors** (Python example):

```python
import smbus
import time

bus = smbus.SMBus(1)
PCA9548A_ADDR = 0x70
SENSOR_ADDR = 0x29  # Default ToF sensor address

# Select Channel 0 (Sensor #1)
bus.write_byte(PCA9548A_ADDR, 0x01)
time.sleep(0.1)
print("Scanning Channel 0:")
# Your sensor code here for sensor #1

# Select Channel 1 (Sensor #2)
bus.write_byte(PCA9548A_ADDR, 0x02)
time.sleep(0.1)
print("Scanning Channel 1:")
# Your sensor code here for sensor #2

# Disable all channels
bus.write_byte(PCA9548A_ADDR, 0x00)
```

---

## PCA9548A Channel Select Values

| Channel | Byte Value | Binary    | Active Sensors |
|---------|-----------|-----------|----------------|
| None    | 0x00      | 0b00000000| None           |
| 0       | 0x01      | 0b00000001| Sensor #1      |
| 1       | 0x02      | 0b00000010| Sensor #2      |
| 0 & 1   | 0x03      | 0b00000011| Both (not recommended) |

**Note**: Only activate one channel at a time to avoid address conflicts.

---

## Troubleshooting

### Issue: PCA9548A not detected
- Check power connections (VIN and GND)
- Verify SDA and SCL are connected to correct Pi pins
- Check if I2C is enabled on Pi
- Try a different I2C address (some boards use 0x71-0x77)

### Issue: Sensors not responding
- Ensure sensors are receiving proper voltage (3.3V or 5V)
- Check channel selection in your code
- Verify sensor connections to correct SD/SC pins
- Test each sensor independently by swapping channels

### Issue: Intermittent readings
- Add pull-up resistors (4.7kΩ) to SDA and SCL lines if not present
- Shorten wire lengths
- Use shielded cables for longer runs
- Check power supply can handle both sensors

---

## Additional Notes

- **Pull-up Resistors**: Most PCA9548A boards have built-in pull-up resistors. If readings are unstable, you may need to add external 4.7kΩ resistors from SDA/SCL to 3.3V.
  
- **Power Considerations**: If both sensors draw significant current, consider using an external power supply instead of drawing from the Pi's 3.3V pin.

- **Cable Length**: Keep I2C cables as short as possible (ideally under 1 meter) to avoid signal degradation.

- **Addressing**: The PCA9548A default address is 0x70. If you need multiple multiplexers, you can change the address using the A0-A2 pins on the board.

---

## Example Python Code for Reading Both Sensors

```python
#!/usr/bin/env python3
import smbus
import time

# Initialize I2C
bus = smbus.SMBus(1)
PCA9548A_ADDR = 0x70

def select_channel(channel):
    """Select PCA9548A channel (0-7)"""
    if 0 <= channel <= 7:
        bus.write_byte(PCA9548A_ADDR, 1 << channel)
    else:
        bus.write_byte(PCA9548A_ADDR, 0)  # Disable all

def read_sensor_1():
    select_channel(0)
    time.sleep(0.05)
    # Add your ToF sensor reading code here
    print("Reading from Sensor #1")

def read_sensor_2():
    select_channel(1)
    time.sleep(0.05)
    # Add your ToF sensor reading code here
    print("Reading from Sensor #2")

# Main loop
try:
    while True:
        read_sensor_1()
        read_sensor_2()
        time.sleep(0.1)
except KeyboardInterrupt:
    select_channel(-1)  # Disable all channels
    print("\nExiting...")
```

---

## Summary

You now have two ToF sensors connected to your Raspberry Pi through the PCA9548A multiplexer:
- **Sensor #1** on Channel 0
- **Sensor #2** on Channel 1

Switch between sensors by selecting the appropriate channel in your code!

