# SPF Project Comparison & Improvement Analysis

## Executive Summary

After analyzing the SPF (See-Point-Fly) project used for drone controls, I've identified several key improvements that can enhance the safety, accuracy, and reliability of our drone tracking system. The SPF project demonstrates sophisticated coordinate transformations, safety mechanisms, and action space management that we can adapt to our ArduPilot-based system.

---

## Key Architectural Differences

### 1. **2D to 3D Projection Method**

#### SPF Approach:
- Uses proper **perspective projection** with FOV-based calculations
- Implements `reverse_project_point()` with:
  - Reference point at **35% from top** (better for drone perspective)
  - Normalized coordinate system (0-1000 scale)
  - Depth-adjusted projection based on vertical position
  - Proper FOV calculations using `tan(fov/2)`

#### Current Approach:
- Simpler angular error calculation
- Direct conversion using `tan(angle)` without proper perspective
- Uses image center as reference

**Recommendation**: Implement SPF-style reverse projection with reference point adjustment.

---

### 2. **Action Space and Command Sequencing**

#### SPF Approach:
- Converts 3D movement vectors to **sequential commands**:
  1. **Yaw first** (rotate to face target)
  2. **Forward movement** (move toward target)
  3. **Vertical movement** (adjust altitude)
- Uses `ActionPoint` dataclass with explicit `yaw_only` flag
- Commands have **durations** (milliseconds) for discrete movements

#### Current Approach:
- Direct PID-based continuous RC override
- All axes controlled simultaneously
- No explicit sequencing or action decomposition

**Recommendation**: Implement action sequencing to improve tracking stability, especially for large corrections.

---

### 3. **Safety Mechanisms**

#### SPF Safety Features:
1. **Yaw-Only Mode**: When depth ≤ 2, prevents forward movement
   - Only rotates to face target
   - Prevents collisions with close objects
   - Uses `yaw_only` flag in `ActionPoint`

2. **Depth Adjustment**:
   - Non-linear scaling: `depth = (vlm_depth/10)^2 * 8` for far objects
   - Special handling for close objects (depth ≤ 2)

3. **Reference Point**: 35% from top accounts for drone camera angle

#### Current Safety Features:
- TOF sensor integration for distance measurement
- Lost target handling with timeouts
- Battery monitoring
- RC override detection

**Recommendation**: Add yaw-only mode for close targets (distance < 2m) to prevent collisions.

---

### 4. **Coordinate System Handling**

#### SPF:
```python
# Normalized coordinates (0-1000 scale)
x_normalized = (point_2d[0] - image_width/2) / (image_width/2)
y_normalized = (reference_y - point_2d[1]) / (image_height/2)

# Reference point at 35% from top
reference_y = image_height * 0.35

# Depth adjustment based on vertical position
depth_factor = 1.0 + (y_normalized * 0.5)
depth = depth * depth_factor

# 3D projection
x = depth * x_normalized * tan(fov_horizontal/2)
z = depth * y_normalized * tan(fov_vertical/2)
y = depth  # Forward distance
```

#### Current:
```python
# Direct angular calculation
angle_yaw = (error_x / camera_width) * fov_h_rad
angle_pitch = (error_y / camera_height) * fov_v_rad

# Simple 3D displacement
lateral_x = distance * tan(angle_yaw)
lateral_y = distance * tan(angle_pitch)
```

**Recommendation**: Implement normalized coordinate system with reference point adjustment.

---

### 5. **Command Execution Strategy**

#### SPF:
- **Discrete commands** with durations
- Queue-based execution
- Action history tracking (deque of last 5 actions)
- Prevents oscillation by tracking opposite actions

#### Current:
- **Continuous RC override** at 20Hz
- PID-based smooth control
- Real-time adjustments

**Current approach is actually better for ArduPilot** because:
- ArduPilot handles smooth control better than discrete commands
- RC override is designed for continuous control
- PID controllers provide smooth, responsive tracking

**Recommendation**: Keep continuous RC override but add action history to prevent oscillation.

---

## Recommended Improvements

### Priority 1: Safety & Accuracy

#### 1.1 Add Yaw-Only Mode for Close Targets
**File**: `rpi/drone_client/controllers/tracking_controller.py`

**Change**: When target distance < 2m, only send yaw commands (no forward/pitch movement)

```python
def _calculate_tracking_commands(self, detection, tof_forward, tof_down):
    distance = self._estimate_distance(detection, tof_forward)
    
    # Safety: Yaw-only mode for close targets
    if distance < 2.0:
        # Only calculate yaw correction
        angle_yaw = (error_x / self.camera_width) * math.radians(self.fov_horizontal)
        yaw_cmd = int(np.clip(self.pid_manager.controllers['yaw'].update(angle_yaw), -100, 100))
        
        return {
            'roll': 0,
            'pitch': 0,  # No forward movement
            'yaw': yaw_cmd,
            'throttle': 50,  # Maintain altitude
            'tracking_active': True,
            'yaw_only': True  # Flag for telemetry
        }
```

#### 1.2 Improve 2D-to-3D Projection
**File**: `rpi/drone_client/controllers/tracking_controller.py`

**Change**: Implement SPF-style reverse projection with reference point

```python
def _reverse_project_point(self, point_2d, depth):
    """SPF-style reverse projection with reference point."""
    center_x = self.camera_width / 2
    reference_y = self.camera_height * 0.35  # 35% from top
    
    # Normalized coordinates
    x_normalized = (point_2d[0] - center_x) / (self.camera_width / 2)
    y_normalized = (reference_y - point_2d[1]) / (self.camera_height / 2)
    
    # Depth adjustment based on vertical position
    depth_factor = 1.0 + (y_normalized * 0.5)
    adjusted_depth = depth * depth_factor
    
    # 3D projection with FOV
    fov_h_rad = math.radians(self.fov_horizontal / 2)
    fov_v_rad = math.radians(self.fov_vertical / 2)
    
    x = adjusted_depth * x_normalized * math.tan(fov_h_rad)
    z = adjusted_depth * y_normalized * math.tan(fov_v_rad)
    y = adjusted_depth
    
    return (x, y, z)
```

---

### Priority 2: Enhanced Control

#### 2.1 Add Action History for Oscillation Prevention
**File**: `rpi/drone_client/controllers/tracking_controller.py`

**Change**: Track recent actions to prevent rapid oscillation

```python
from collections import deque

class TrackingController:
    def __init__(self, ...):
        # ... existing code ...
        self.action_history = deque(maxlen=5)  # Track last 5 actions
        
    def _calculate_tracking_commands(self, ...):
        # ... calculate commands ...
        
        # Check for oscillation (rapid direction changes)
        if len(self.action_history) >= 2:
            last_action = self.action_history[-1]
            # If yaw direction changed rapidly, reduce gain
            if abs(yaw_cmd) > 0 and abs(last_action.get('yaw', 0)) > 0:
                if (yaw_cmd * last_action['yaw']) < 0:  # Opposite signs
                    yaw_cmd *= 0.5  # Reduce by 50%
        
        self.action_history.append({
            'roll': roll_cmd,
            'pitch': pitch_cmd,
            'yaw': yaw_cmd
        })
```

#### 2.2 Improved Depth Estimation
**File**: `rpi/drone_client/controllers/tracking_controller.py`

**Change**: Use SPF-style depth adjustment for better distance handling

```python
def _estimate_distance(self, detection, tof_forward):
    """Enhanced distance estimation with SPF-style adjustment."""
    # Use TOF if available
    if tof_forward is not None and 0.5 <= tof_forward <= 4.0:
        return tof_forward
    
    # Estimate from bbox with improved calibration
    bbox = detection['bbox']
    bbox_height = bbox[3] - bbox[1]
    height_ratio = bbox_height / self.camera_height
    
    # SPF-style non-linear scaling for better accuracy
    if height_ratio > 0.5:
        return 1.0
    elif height_ratio > 0.3:
        return 2.0
    elif height_ratio > 0.2:
        return 3.0
    elif height_ratio > 0.15:
        # Use non-linear scaling for medium distances
        return 3.0 + (0.2 - height_ratio) * 20  # Smooth interpolation
    elif height_ratio > 0.1:
        return 5.0
    else:
        return 8.0
```

---

### Priority 3: Code Quality & Architecture

#### 3.1 Extract Action Projection to Separate Module
**Recommendation**: Create `rpi/drone_client/vision/action_projector.py` similar to SPF

**Benefits**:
- Reusable projection logic
- Easier to test and calibrate
- Separation of concerns

#### 3.2 Add Visualization & Debugging
**Recommendation**: Add visualization similar to SPF's decision saving

**File**: `rpi/drone_client/vision/action_projector.py` (new)

```python
def visualize_action(self, frame, action, detection):
    """Visualize action on frame for debugging."""
    viz_frame = frame.copy()
    
    # Draw detection box
    bbox = detection['bbox']
    cv2.rectangle(viz_frame, (bbox[0], bbox[1]), 
                  (bbox[0]+bbox[2], bbox[1]+bbox[3]), (0, 255, 0), 2)
    
    # Draw action vector
    center_x, center_y = detection['center']
    cv2.arrowedLine(viz_frame, 
                    (int(center_x), int(center_y)),
                    (int(center_x + action['dx']*10), int(center_y - action['dz']*10)),
                    (255, 0, 0), 2)
    
    # Add text info
    cv2.putText(viz_frame, f"Dist: {action['distance']:.1f}m", 
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    
    return viz_frame
```

---

## Implementation Priority

### Phase 1: Safety (Before Outdoor Testing)
1. ✅ Add yaw-only mode for close targets (< 2m)
2. ✅ Improve depth estimation with TOF fallback
3. ✅ Add action history for oscillation prevention

### Phase 2: Accuracy (After Initial Testing)
1. Implement SPF-style reverse projection
2. Add reference point adjustment (35% from top)
3. Improve coordinate normalization

### Phase 3: Architecture (Long-term)
1. Extract action projection to separate module
2. Add visualization and debugging tools
3. Implement action space class for command sequencing (optional)

---

## What NOT to Change

### Keep Current Approach For:
1. **Continuous RC Override**: Better for ArduPilot than discrete commands
2. **PID Controllers**: Provide smooth, responsive control
3. **Async Architecture**: Better performance than SPF's threading
4. **MAVLink Integration**: More robust than Tello SDK

---

## Testing Recommendations

Before implementing changes:
1. **Baseline Test**: Record current tracking performance
2. **Incremental Changes**: Implement one improvement at a time
3. **A/B Testing**: Compare old vs new for each change
4. **Safety First**: Test yaw-only mode extensively before enabling forward movement

---

## Conclusion

The SPF project provides excellent insights into:
- Proper 2D-to-3D coordinate transformation
- Safety mechanisms for close-range operation
- Depth estimation improvements

However, our current architecture with:
- Continuous RC override
- PID controllers
- Async event loops
- MAVLink integration

Is actually **better suited for ArduPilot** than SPF's discrete command approach.

**Key Takeaway**: Adopt SPF's **mathematical improvements** (projection, depth estimation) while keeping our **control architecture** (continuous PID, RC override).

---

## Files to Modify

1. `rpi/drone_client/controllers/tracking_controller.py` - Add yaw-only mode, improve projection
2. `rpi/drone_client/controllers/tracking_controller.py` - Add action history
3. `rpi/drone_client/config/production.yaml` - Add safety thresholds
4. `rpi/drone_client/vision/action_projector.py` - NEW: Extract projection logic (optional)

---

*Analysis Date: 2024*
*Based on: SPF Project (See-Point-Fly) Comparison*

