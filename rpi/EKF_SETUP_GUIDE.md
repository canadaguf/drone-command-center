# ArduPilot EKF Setup Guide - ToF Sensor Integration

## Overview

ArduPilot's Extended Kalman Filter (EKF) is a sophisticated sensor fusion algorithm that combines data from multiple sensors (gyros, accelerometers, compass, GPS, barometer, rangefinders) to estimate the vehicle's position, velocity, and orientation with high accuracy.

**Current Status**: Your ToF sensors are currently only used by the companion computer (Raspberry Pi) for local altitude control. They are NOT being sent to ArduPilot's EKF, which means the flight controller is missing valuable altitude data.

## Why Integrate ToF Sensors into EKF?

1. **Better Altitude Estimation**: EKF can fuse ToF data with barometer and other sensors for more accurate altitude
2. **Indoor Operation**: Essential for GPS-denied environments where GPS altitude is unavailable
3. **Smoother Control**: EKF's filtered altitude is more stable than raw sensor readings
4. **Redundancy**: Multiple altitude sources improve reliability

## ArduPilot EKF Architecture

The EKF (EKF3 in ArduPilot) can fuse:
- **IMU**: Gyroscopes and accelerometers (always used)
- **GPS**: Position and velocity (when available)
- **Barometer**: Pressure-based altitude
- **Compass**: Heading
- **Rangefinders**: Distance sensors (ToF, LiDAR, ultrasonic)
- **Optical Flow**: Ground velocity (for position hold without GPS)

## Step 1: Send ToF Data to ArduPilot via MAVLink

### MAVLink DISTANCE_SENSOR Message

ArduPilot receives rangefinder data via the `DISTANCE_SENSOR` MAVLink message:

```
MAVLink Message ID: 132 (DISTANCE_SENSOR)
Fields:
- time_boot_ms: Timestamp
- min_distance: Minimum measurable distance (mm)
- max_distance: Maximum measurable distance (mm)
- current_distance: Current distance reading (mm)
- type: Sensor type (MAV_DISTANCE_SENSOR_LASER = 1 for ToF)
- id: Sensor ID (0-7, use 0 for downward, 1 for forward)
- orientation: Sensor orientation (MAV_SENSOR_ORIENTATION_DOWNWARD = 25, MAV_SENSOR_ORIENTATION_FORWARD = 0)
- covariance: Measurement covariance (0 = unknown/ignored)
```

### Implementation Required

1. **Add DISTANCE_SENSOR message sender** to `drone_controller.py`
2. **Send bottom ToF sensor data** at 10-20 Hz (matching sensor reading rate)
3. **Send forward ToF sensor data** optionally (for obstacle avoidance)

## Step 2: Configure ArduPilot Parameters

### Rangefinder Configuration

Connect to ArduPilot via Mission Planner or QGroundControl and set:

```
# Bottom ToF Sensor (Downward-facing)
RNGFND1_TYPE = 11          # Generic MAVLink (for companion computer)
RNGFND1_MAX_CM = 400       # 4 meters max range
RNGFND1_MIN_CM = 4         # 4cm min range (VL53L1X minimum)
RNGFND1_ORIENT = 25        # Downward-facing
RNGFND1_PIN = -1           # Not connected to FC directly (via MAVLink)
RNGFND1_SCALING = 1.0      # Scaling factor (1.0 for meters)

# Forward ToF Sensor (Optional, for obstacle avoidance)
RNGFND2_TYPE = 11          # Generic MAVLink
RNGFND2_MAX_CM = 400       # 4 meters max range
RNGFND2_MIN_CM = 30        # 30cm min range
RNGFND2_ORIENT = 0         # Forward-facing
RNGFND2_PIN = -1
RNGFND2_SCALING = 1.0
```

### EKF Configuration for Indoor/GPS-Denied Operation

```
# Enable EKF3
EK3_ENABLE = 1

# Use Rangefinder for Altitude (Z position)
EK3_SRC1_POSZ = 2          # 0=None, 1=Baro, 2=Rangefinder, 3=GPS
EK3_SRC1_VELZ = 0          # Vertical velocity source (0=Baro, 1=GPS)

# For GPS-denied operation
EK3_GPS_TYPE = 3            # 0=GPS, 1=NoGPS, 2=Beacon, 3=NonGPS
EK3_ALT_SOURCE = 2          # Altitude source: 0=Baro, 1=GPS, 2=Rangefinder

# Rangefinder fusion settings
EK3_RNG_USE_HGT = 100       # Use rangefinder for height estimation (0-100%, 100=always)
EK3_RNG_DELAY_MS = 0        # Sensor delay compensation (ms)
EK3_RNG_GATE = 5            # Measurement gate size (standard deviations)

# Barometer still used for velocity estimation
EK3_BARO_GATE = 5           # Barometer gate size
```

### Altitude Hold Mode Configuration

```
# ALT_HOLD mode uses EKF altitude
AHRS_EKF_TYPE = 3           # Use EKF3 (not DCM)
```

## Step 3: Verify Integration

### Pre-Flight Checks

1. **Check Rangefinder Status**:
   - In Mission Planner: Flight Data → Status → Rangefinder
   - Should show valid distance readings updating at ~10 Hz

2. **Check EKF Status**:
   - In Mission Planner: Flight Data → Status → EKF Status
   - Verify EKF is healthy and using rangefinder data

3. **Test Sensor Data**:
   - Hold drone at known heights (e.g., 0.5m, 1.0m, 1.5m)
   - Verify EKF altitude matches ToF sensor reading

### In-Flight Monitoring

- Monitor EKF altitude vs ToF sensor reading
- Check EKF health flags
- Verify smooth altitude hold in ALT_HOLD mode

## Step 4: Troubleshooting

### Common Issues

1. **EKF Not Using Rangefinder**:
   - Check `EK3_SRC1_POSZ = 2` is set
   - Verify `RNGFND1_TYPE = 11` (MAVLink)
   - Check DISTANCE_SENSOR messages are being sent

2. **Erratic Altitude**:
   - Increase `EK3_RNG_GATE` (reduce sensitivity to outliers)
   - Check ToF sensor readings are stable
   - Verify sensor orientation is correct

3. **Altitude Drift**:
   - EKF may need time to converge
   - Check barometer calibration
   - Verify ToF sensor accuracy

## Benefits of EKF Integration

1. **Smoother Altitude Control**: EKF filters noise from ToF sensor
2. **Better Indoor Performance**: Essential for GPS-denied environments
3. **Redundancy**: Multiple altitude sources (baro + ToF)
4. **Improved Safety**: EKF can detect sensor failures

## Next Steps

1. Implement DISTANCE_SENSOR message sender in drone_controller.py
2. Configure ArduPilot parameters as above
3. Test integration on ground first
4. Verify EKF is using rangefinder data
5. Test in controlled indoor environment

## References

- ArduPilot EKF3 Documentation: https://ardupilot.org/dev/docs/ekf3-overview.html
- Rangefinder Setup: https://ardupilot.org/copter/docs/common-rangefinder-landingpage.html
- MAVLink DISTANCE_SENSOR: https://mavlink.io/en/messages/common.html#DISTANCE_SENSOR
- EKF3 Parameters: https://ardupilot.org/dev/docs/parameters.html#ekf3-parameters

