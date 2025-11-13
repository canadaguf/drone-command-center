# I2C Error Analysis: ToF Sensor Failures During Motor Operation

## Problem Summary
- **Low throttle**: Minimal/no errors
- **Medium throttle**: Some errors appear
- **High throttle**: Continuous error loop (EAGAIN - Resource temporarily unavailable)
- **Hardware setup**: Multiplexer board on double-sided tape on carbon fiber frame

## Software-Side Analysis

### Current Issues

1. **Insufficient Retry Delays**
   - Current: 10ms, 20ms, 40ms exponential backoff
   - Problem: During high EMI, I2C bus may be unavailable for longer periods
   - Solution: Increase delays and add adaptive backoff

2. **No Circuit Breaker Pattern**
   - Problem: Continuous retries during high EMI create a loop
   - Solution: Add exponential backoff that increases when errors persist

3. **Bus Lock Contention**
   - Problem: Reading loop holds lock for entire sensor read cycle
   - Solution: Release lock between operations, use shorter critical sections

4. **No I2C Bus Recovery**
   - Problem: If I2C bus gets stuck, no recovery mechanism
   - Solution: Add bus reset/recovery when errors persist

### Recommended Software Fixes

1. **Adaptive Retry Delays**
   - Start with 10ms, increase exponentially up to 100ms
   - Track consecutive failures and increase base delay
   - Reset delay on successful read

2. **Circuit Breaker**
   - After N consecutive failures, pause reads for that sensor
   - Gradually resume with longer delays
   - Prevents continuous error loops

3. **I2C Bus Reset**
   - If errors persist, attempt to reset I2C bus
   - Reinitialize sensors if bus reset succeeds

4. **Shorter Critical Sections**
   - Release bus lock between channel select and data read
   - Reduces lock contention

## Hardware-Side Analysis

### EMI (Electromagnetic Interference)

**Primary Cause**: Brushless motors generate strong EMI at high RPM
- **Frequency**: Motor PWM creates harmonics in kHz-MHz range
- **I2C Frequency**: 100kHz (standard) or 400kHz (fast mode)
- **Problem**: EMI can couple into I2C lines, corrupting signals

**Evidence**:
- Errors increase with throttle (motor RPM)
- EAGAIN suggests I2C controller detects corrupted signals
- Continuous loop at high throttle = sustained EMI

### Power Supply Issues

**Potential Problems**:
1. **Voltage Droop**: High current draw causes voltage fluctuations
2. **Ripple**: PWM switching creates power supply noise
3. **Ground Bounce**: Current spikes cause ground potential shifts

**Symptoms**:
- I2C devices may reset or lose communication
- Signal levels may drop below thresholds

### Carbon Fiber Frame Effects

**Issues**:
1. **Conductive Frame**: Carbon fiber is conductive (acts like ground plane)
2. **Capacitive Coupling**: Board near frame creates capacitance
3. **Antenna Effect**: Frame can act as antenna, picking up EMI
4. **Ground Loops**: If frame is grounded, creates ground loop potential

**Even with Double-Sided Tape**:
- Tape provides insulation but doesn't eliminate:
  - Capacitive coupling (through air/tape)
  - Magnetic field coupling
  - Ground potential differences

### I2C Bus Characteristics

**Vulnerabilities**:
- **Open-drain lines**: SDA/SCL are pulled high, sensitive to noise
- **No error correction**: Single bit error = communication failure
- **Long bus**: More susceptible to EMI pickup
- **No shielding**: Unshielded wires act as antennas

## Hardware Solutions (Priority Order)

### 1. **I2C Bus Shielding** (HIGHEST PRIORITY)
```
- Use shielded I2C cables (foil shield + drain wire)
- Connect shield to ground at ONE point only (avoid ground loops)
- Keep I2C wires away from motor wires
- Route I2C wires separately from power wires
```

### 2. **Power Supply Filtering**
```
- Add ferrite cores to I2C power lines
- Add 100nF ceramic capacitor near each I2C device
- Add 10µF electrolytic capacitor for bulk filtering
- Use separate power rail for I2C devices if possible
```

### 3. **Physical Separation**
```
- Move multiplexer board away from motors
- Increase distance from carbon fiber frame
- Use standoffs instead of tape (creates air gap)
- Mount on non-conductive material (plastic/wood)
```

### 4. **I2C Bus Pull-up Resistors**
```
- Check pull-up resistor values (typically 2.2kΩ-10kΩ)
- Lower values = stronger signal but more current
- Higher values = weaker signal but less current
- Consider active pull-up circuits for noisy environments
```

### 5. **Grounding Strategy**
```
- Single-point ground for all I2C devices
- Avoid ground loops
- Connect I2C ground to Pi ground, not frame
- Keep I2C ground separate from motor ground if possible
```

### 6. **I2C Bus Speed Reduction**
```
- Reduce I2C speed from 400kHz to 100kHz (more tolerant to noise)
- Add delays between I2C operations
- This is a software change but helps with hardware issues
```

### 7. **Twisted Pair Wiring**
```
- Use twisted pair for SDA/SCL (reduces EMI pickup)
- Keep wires close together (reduces loop area)
- Minimize wire length
```

### 8. **EMI Suppression on Motors**
```
- Add ferrite cores to motor wires
- Use shielded motor wires
- Add capacitors across motor phases
- Keep motor wires away from I2C bus
```

## Software Improvements

### 1. Adaptive Retry with Circuit Breaker

```python
# Track error rate per sensor
self.sensor_error_rate = {}  # errors per second
self.sensor_circuit_breaker = {}  # paused until timestamp

# In reading loop:
if sensor in circuit_breaker and time.time() < circuit_breaker[sensor]:
    continue  # Skip this sensor

# After failure:
error_rate[sensor] = error_rate.get(sensor, 0) + 1
if error_rate[sensor] > threshold:
    # Circuit breaker: pause for exponential backoff
    pause_time = min(2.0 ** error_rate[sensor], 10.0)
    circuit_breaker[sensor] = time.time() + pause_time
```

### 2. I2C Bus Recovery

```python
def _recover_i2c_bus(self):
    """Attempt to recover I2C bus after persistent failures."""
    try:
        # Close and reopen I2C bus
        self.bus.close()
        time.sleep(0.1)
        self.bus = smbus2.SMBus(1)
        
        # Reinitialize sensors
        for name, sensor in self.sensors.items():
            sensor.initialize()
    except Exception as e:
        logger.error(f"I2C bus recovery failed: {e}")
```

### 3. Longer Retry Delays During High EMI

```python
# Increase delays when errors are frequent
base_delay = 0.01
if self.sensor_error_rate.get(name, 0) > 5:  # High error rate
    base_delay = 0.05  # Use longer delays

delay = base_delay * (2 ** attempt)
```

## Testing Recommendations

1. **Test with I2C speed reduction**:
   ```python
   # In busio.I2C initialization
   i2c = busio.I2C(board.SCL, board.SDA, frequency=100000)  # 100kHz instead of 400kHz
   ```

2. **Test with physical separation**:
   - Move multiplexer board further from frame
   - Test with board on non-conductive standoffs

3. **Test power supply filtering**:
   - Add capacitors near I2C devices
   - Use separate power supply for I2C

4. **Test with shielded cables**:
   - Replace I2C wires with shielded versions
   - Properly ground shield

## Immediate Actions

### Software (Quick Fix)
1. Increase retry delays (10ms → 50ms base)
2. Add circuit breaker to prevent continuous loops
3. Reduce I2C speed to 100kHz

### Hardware (Best Fix)
1. Add ferrite cores to I2C lines
2. Move multiplexer board away from frame (use standoffs)
3. Add power supply filtering capacitors
4. Use shielded I2C cables
5. Keep I2C wires away from motor wires

## Expected Results

- **With software fixes only**: Reduced error rate, but errors will still occur
- **With hardware fixes**: Near-zero errors even at high throttle
- **Combined approach**: Most robust solution

