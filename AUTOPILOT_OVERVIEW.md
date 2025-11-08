# Autonomous Drone Autopilot Overview

## Executive Summary

This document provides a comprehensive overview of the autonomous drone autopilot system, with special focus on **distance measurement** and **how distance data is used for flight control**. The system integrates vision-based person tracking with Time-of-Flight (TOF) sensors for robust obstacle avoidance and precise altitude control.

---

## System Architecture

### Core Components

1. **Vision System** - YOLO-based person detection and tracking
2. **TOF Sensors** - VL53L1X distance sensors (forward-facing and downward-facing)
3. **Tracking Controller** - Converts 2D detections to 3D movement commands
4. **PID Controllers** - Smooth control for yaw, pitch, roll, and distance
5. **MAVLink Controller** - Communication with ArduPilot flight controller
6. **Safety Systems** - Obstacle avoidance, altitude limits, battery monitoring

---

## Distance Measurement System

### Overview

The autopilot uses **two complementary methods** for distance measurement:

1. **TOF Sensors (Primary)** - Direct distance measurement using VL53L1X sensors
2. **Vision-Based Estimation (Fallback)** - Distance estimation from bounding box size

### TOF Sensor Configuration

- **Channel 0 (Forward-facing)**: Measures distance to obstacles/targets ahead
- **Channel 1 (Downward-facing)**: Measures altitude above ground/obstacles
- **Multiplexer**: TCA9548A I2C multiplexer allows multiple sensors on same I2C bus
- **Range**: 0.3m to 4.0m (most reliable range)
- **Update Rate**: 10 Hz (100ms intervals)
- **Units**: Centimeters from sensor, converted to meters internally

### Distance Calculation Flow

```
┌─────────────────────────────────────────────────────────────┐
│                    Distance Measurement                       │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
        ┌─────────────────────────────────────┐
        │   Forward TOF Sensor Reading        │
        │   (Channel 0, front-facing)         │
        └─────────────────────────────────────┘
                      │
        ┌─────────────┴─────────────┐
        │                           │
        ▼                           ▼
  ┌──────────┐              ┌──────────────┐
  │ Valid?   │              │ Use TOF      │
  │ (0.3-4m) │              │ Distance    │
  └──────────┘              └──────────────┘
        │                           │
        │ NO                        │ YES
        ▼                           │
  ┌─────────────────────────────┐  │
  │ Vision-Based Estimation     │  │
  │ (Bounding box size)         │  │
  └─────────────────────────────┘  │
        │                           │
        └───────────┬───────────────┘
                    ▼
        ┌───────────────────────┐
        │   Final Distance      │
        │   (in meters)         │
        └───────────────────────┘
```

### TOF Distance Processing

**Location**: `rpi/drone_client/sensors/tof_manager.py`

1. **Reading Process**:
   - Select multiplexer channel (0 for forward, 1 for down)
   - Wait 20ms for channel switching
   - Check if sensor data is ready
   - Read distance in centimeters
   - Convert to meters (divide by 100)
   - Validate reading (0.3m to max_range)
   - Clear interrupt for next reading

2. **Distance Validation**:
   ```python
   # Valid range: 0.3m to max_range (typically 4.0m)
   if distance_cm <= 0 or distance_cm > (max_range * 100):
       return None  # Invalid reading
   ```

3. **Thread Safety**:
   - Continuous reading loop runs in separate thread
   - Thread-safe access via locks
   - Latest readings cached for fast access

### Vision-Based Distance Estimation

**Location**: `rpi/drone_client/controllers/tracking_controller.py::_estimate_distance()`

When TOF sensor is unavailable or out of range, the system falls back to vision-based estimation:

| Bounding Box Height Ratio | Estimated Distance |
|---------------------------|-------------------|
| > 0.5 (50% of frame)      | 1.0m (very close) |
| > 0.3 (30% of frame)      | 2.0m (close)     |
| > 0.2 (20% of frame)      | 3.0m (medium)    |
| > 0.15 (15% of frame)     | 3.0-5.0m (interpolated) |
| > 0.1 (10% of frame)      | 5.0m (medium-far) |
| > 0.05 (5% of frame)      | 5.0-8.0m (interpolated) |
| ≤ 0.05                    | 8.0m (very far)   |

**Priority**: TOF sensor reading is **always preferred** when available and valid (0.3m - 4.0m range).

---

## Flight Control System

### How Distance Data is Used

Distance measurements are **actively used** in multiple control loops:

#### 1. **Obstacle Avoidance (Forward TOF)**

**Location**: `tracking_controller.py::_calculate_tracking_commands()`

**Thresholds**:
- `emergency_stop_threshold`: 0.8m - Immediate stop, all movement blocked
- `obstacle_threshold_forward`: 1.5m - Forward movement blocked, yaw-only mode

**Behavior**:
```python
if tof_forward < 0.8m:
    # EMERGENCY STOP
    return emergency_stop_commands()  # All movement = 0
    
elif tof_forward < 1.5m:
    # Prevent forward movement
    obstacle_detected = True
    pitch_cmd = 0  # Block forward movement
    
if obstacle_detected and pitch_cmd > 0:
    pitch_cmd = 0  # Safety override
```

**Result**: Forward TOF sensor **directly prevents collisions** by blocking forward movement commands.

#### 2. **Altitude Control (Downward TOF)**

**Location**: `tracking_controller.py::_calculate_altitude_adjustment()`

**Thresholds**:
- `obstacle_threshold_down`: 0.5m - Emergency climb
- `min_height`: 0.5m (config) - Minimum safe altitude
- `max_height`: 2.0m (config) - Maximum altitude

**Behavior**:
```python
if tof_down < 0.5m:
    adjustment = 50  # Maximum climb (emergency)
elif tof_down < 0.8m:
    adjustment = max(adjustment, 30)  # Strong climb
elif tof_down < 1.2m:
    adjustment = max(adjustment, 10)  # Gentle climb
elif tof_down > 3.0m:
    adjustment = min(adjustment, 0)  # Allow descent
```

**Result**: Downward TOF sensor **actively controls altitude** to maintain safe distance from ground.

#### 3. **Target Distance Control**

**Location**: `tracking_controller.py::_calculate_tracking_commands()`

**Distance Modes**:
- `close`: 2.0m target distance
- `medium`: 4.0m target distance (default)
- `far`: 6.0m target distance

**PID Control**:
```python
pid_errors = {
    'distance': distance - target_distance  # Error signal
}
pid_outputs = pid_manager.update(pid_errors)
pitch_cmd = pid_outputs['pitch']  # Adjust forward/backward
```

**Result**: Distance measurement **directly controls** forward/backward movement to maintain target distance.

#### 4. **Yaw-Only Safety Mode**

**Location**: `tracking_controller.py::_calculate_tracking_commands()`

**Trigger Conditions**:
- Target distance < `yaw_only_threshold` (2.0m)
- OR forward obstacle detected (< 1.5m)

**Behavior**:
```python
if distance < 2.0m or obstacle_detected_forward:
    # YAW ONLY MODE
    roll_cmd = 0      # No lateral movement
    pitch_cmd = 0     # No forward movement
    yaw_cmd = ...     # Only yaw correction
    throttle_cmd = ... # Altitude adjustment still allowed
```

**Result**: Prevents forward movement when too close to target or obstacles.

---

## Control Flow Diagram

```
┌─────────────────────────────────────────────────────────────┐
│                    Vision Loop (20 Hz)                       │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
        ┌─────────────────────────────────────┐
        │   YOLO Detection + Person Tracking  │
        └─────────────────────────────────────┘
                      │
                      ▼
        ┌─────────────────────────────────────┐
        │   Get TOF Readings                  │
        │   - Forward: tof_forward (m)        │
        │   - Down: tof_down (m)              │
        └─────────────────────────────────────┘
                      │
                      ▼
        ┌─────────────────────────────────────┐
        │   Calculate Distance                │
        │   Priority: TOF > Vision            │
        └─────────────────────────────────────┘
                      │
        ┌─────────────┴─────────────┐
        │                           │
        ▼                           ▼
┌───────────────┐          ┌───────────────┐
│ Obstacle      │          │ Normal         │
│ Avoidance     │          │ Tracking       │
│ Check         │          │                │
└───────────────┘          └───────────────┘
        │                           │
        ▼                           ▼
┌───────────────┐          ┌───────────────┐
│ Emergency?    │          │ Calculate      │
│ (< 0.8m)      │          │ PID Errors     │
└───────────────┘          └───────────────┘
        │                           │
        │ YES                       │
        ▼                           │
┌───────────────┐                  │
│ STOP ALL      │                  │
│ MOVEMENT      │                  │
└───────────────┘                  │
        │                           │
        └───────────┬───────────────┘
                    ▼
        ┌───────────────────────┐
        │   Altitude Control    │
        │   (Uses tof_down)     │
        └───────────────────────┘
                    │
                    ▼
        ┌───────────────────────┐
        │   Generate RC        │
        │   Commands            │
        │   (roll, pitch, yaw,  │
        │    throttle)          │
        └───────────────────────┘
                    │
                    ▼
        ┌───────────────────────┐
        │   Send to Flight      │
        │   Controller          │
        │   (MAVLink RC Override)│
        └───────────────────────┘
```

---

## Safety Systems

### ⚠️ CRITICAL SAFETY: Altitude Maintenance

**IMPORTANT**: During emergency stop and hover, the system **ALWAYS maintains hover throttle (50)** to prevent the drone from falling. The throttle is never set below 50 (hover) except during intentional landing.

**Throttle Values**:
- `throttle = 0` → 1000 RC (motors off - **DANGEROUS**, causes fall)
- `throttle = 50` → 1500 RC (hover - **SAFE**, maintains altitude)
- `throttle = 100` → 2000 RC (maximum climb)

**Emergency Stop Behavior**:
- Horizontal movement: **BLOCKED** (roll=0, pitch=0, yaw=0)
- Vertical movement: **MAINTAINED** (throttle ≥ 50)
- Ground avoidance: Allows climb if too close to ground

### 1. Forward Obstacle Avoidance

**Sensor**: Forward TOF (Channel 0)
**Thresholds**:
- Emergency stop: < 0.8m
- Obstacle warning: < 1.5m

**Actions**:
- < 0.8m: Complete stop, all horizontal movement blocked
  - **CRITICAL**: Throttle maintained at hover (50) or higher to prevent falling
  - Only allows climb if ground too close, never descent
- < 1.5m: Forward movement blocked, yaw-only mode

### 2. Altitude Safety

**Sensor**: Downward TOF (Channel 1)
**Thresholds**:
- Emergency climb: < 0.5m
- Low altitude warning: < 0.8m
- Minimum safe: 0.5m (config)
- Maximum safe: 2.0m (config)

**Actions**:
- < 0.5m: Maximum climb (50% throttle)
- < 0.8m: Strong climb (30% throttle)
- < min_height: Enforce minimum altitude
- > max_height: Limit maximum altitude

### 3. Target Distance Safety

**Method**: Combined TOF + Vision
**Threshold**: 2.0m (yaw_only_threshold)

**Actions**:
- < 2.0m: Yaw-only mode (no forward movement)
- Prevents collision with tracked target

---

## Distance Data Usage Summary

| Use Case | Sensor | Threshold | Action | Status |
|----------|--------|-----------|--------|--------|
| **Obstacle Avoidance** | Forward TOF | 1.5m | Block forward movement | ✅ **ACTIVE** |
| **Emergency Stop** | Forward TOF | 0.8m | Stop horizontal movement, maintain altitude | ✅ **ACTIVE** |
| **Altitude Control** | Downward TOF | 0.5-3.0m | Adjust throttle | ✅ **ACTIVE** |
| **Target Distance** | Forward TOF + Vision | 2.0-6.0m | PID control pitch | ✅ **ACTIVE** |
| **Yaw-Only Mode** | Forward TOF + Vision | < 2.0m | Disable forward movement | ✅ **ACTIVE** |
| **Ground Avoidance** | Downward TOF | < 0.5m | Emergency climb | ✅ **ACTIVE** |

**✅ All distance data is actively used in flight control!**

---

## Configuration

### TOF Sensor Configuration

**File**: `rpi/drone_client/config/production.yaml`

```yaml
tof_sensors:
  multiplexer_address: 0x70
  sensors:
    - name: "forward"
      channel: 0      # Front-facing
      direction: "front"
      max_range: 4.0
    - name: "down"
      channel: 1      # Downward-facing
      direction: "down"
      max_range: 4.0
```

### Safety Thresholds

```yaml
tracking:
  safety:
    yaw_only_threshold: 2.0              # Target distance threshold
    obstacle_threshold_forward: 1.5      # Forward obstacle detection
    obstacle_threshold_down: 0.5         # Ground/obstacle below
    emergency_stop_threshold: 0.8        # Emergency stop distance
```

---

## Testing and Validation

### TOF Sensor Testing

Test scripts available:
- `rpi/test_snippets/worked_best/tof_test_multiplexer.py` - Full multiplexer test
- `rpi/test_snippets/worked_best/tof_test_1.py` - Basic sensor test

**Expected Behavior**:
- Channel 0 (forward): Reads distance to objects ahead
- Channel 1 (down): Reads altitude above ground
- Readings in centimeters, converted to meters internally
- Update rate: ~10 Hz

### Integration Testing

1. **Obstacle Avoidance Test**:
   - Place obstacle < 1.5m in front
   - Verify forward movement is blocked
   - Verify yaw-only mode activates

2. **Altitude Control Test**:
   - Fly at low altitude (< 1.0m)
   - Verify automatic climb when < 0.5m
   - Verify altitude maintained within min/max bounds

3. **Distance Tracking Test**:
   - Track person at various distances
   - Verify TOF reading used when available
   - Verify fallback to vision estimation

---

## Key Takeaways

1. **Distance is Actively Used**: All distance measurements (TOF and vision) are **directly integrated** into flight control decisions.

2. **Multiple Safety Layers**: 
   - Forward obstacle avoidance
   - Altitude safety limits
   - Target distance safety
   - Emergency stop system

3. **Sensor Priority**: TOF sensors take priority over vision-based estimation when available and valid.

4. **Real-time Control**: Distance readings update at 10 Hz and are used in every control cycle (20 Hz vision loop).

5. **Fail-Safe Design**: System gracefully degrades to vision-only mode if TOF sensors fail.

---

## Future Enhancements

Potential improvements:
1. **Sensor Fusion**: Combine TOF + Vision for more robust distance estimation
2. **Predictive Obstacle Avoidance**: Use velocity estimates to predict collisions
3. **Multi-directional Obstacle Avoidance**: Add left/right/back sensors
4. **Adaptive Thresholds**: Adjust thresholds based on flight conditions
5. **Kalman Filtering**: Smooth distance readings for better control

---

## Conclusion

The autopilot system **actively uses distance data** from TOF sensors for:
- ✅ Obstacle avoidance (forward sensor)
- ✅ Altitude control (downward sensor)
- ✅ Target distance maintenance (forward sensor + vision)
- ✅ Safety limits and emergency stops

Distance measurements are **not just collected** - they are **integrated into every control decision** to ensure safe and effective autonomous flight.

