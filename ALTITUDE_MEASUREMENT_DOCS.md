# Altitude Measurement Documentation

## How the Drone Measures Height

### Primary Method: MAVLink GLOBAL_POSITION_INT

The drone gets its altitude from the flight controller (ArduPilot) via MAVLink telemetry:

**Message Type**: `GLOBAL_POSITION_INT`

**Altitude Fields**:
1. **`altitude`** (Absolute/MSL): 
   - Mean Sea Level altitude in millimeters
   - Converted to meters: `altitude / 1000.0`
   - Uses barometer + GPS fusion
   - Range: Typically -500m to 9000m

2. **`relative_alt`** (Relative to Home):
   - Altitude relative to home position (takeoff point) in millimeters
   - Converted to meters: `relative_alt / 1000.0`
   - This is what's displayed in telemetry as "Relative Alt"
   - Range: 0m to max flight altitude

**Location**: `rpi/drone_client/controllers/mavlink_controller.py` lines 298-304

```python
if msg.get_type() == 'GLOBAL_POSITION_INT':
    self._telemetry_cache.update({
        'lat': msg.lat / 1e7,
        'lon': msg.lon / 1e7,
        'altitude': msg.alt / 1000.0,        # Absolute altitude (MSL)
        'relative_alt': msg.relative_alt / 1000.0  # Relative to home
    })
```

### Secondary Method: TOF Sensor (Downward)

The downward-facing Time-of-Flight (TOF) sensor provides distance to ground:

**Sensor**: VL53L0X/VL53L1X TOF sensor pointing downward

**Usage**:
- Primary: Obstacle detection below drone
- Secondary: Altitude verification/cross-check
- Not used as primary altitude source (TOF has limited range: ~4m max)

**Location**: `rpi/drone_client/sensors/tof_manager.py`

**Telemetry Field**: `tof_down` (in meters)

### Why MAVLink is Primary

1. **Accuracy**: Barometer + GPS fusion is more accurate than TOF
2. **Range**: Works at any altitude (TOF limited to ~4m)
3. **Reliability**: Barometer works in all conditions (TOF can fail in bright sunlight)
4. **Flight Controller Integration**: ArduPilot uses this for position hold, altitude control, etc.

---

## Takeoff Altitude Configuration

### Current Implementation

**Default**: 2.0 meters

**Location**: 
- Hardcoded: `rpi/drone_client/main.py` line 440 (OLD - now configurable)
- Configurable: `rpi/drone_client/config/production.yaml` → `mavlink.takeoff_altitude`

### Configuration

```yaml
# rpi/drone_client/config/production.yaml
mavlink:
  connection: "/dev/ttyAMA0"
  baud: 256000
  takeoff_altitude: 2.0  # Default takeoff altitude in meters
```

### How It Works

1. User clicks "Liftoff" button in frontend
2. Frontend sends `takeoff` command via WebSocket
3. Backend forwards to drone client
4. Drone client reads `takeoff_altitude` from config
5. Sends `MAV_CMD_NAV_TAKEOFF` with altitude parameter
6. ArduPilot climbs to specified altitude in GUIDED mode

**Code Flow**:
```
Frontend (ControlPanel.jsx)
  → sendCommand('takeoff')
  → Backend (main.py) 
  → Drone Client (main.py: _on_command_received)
  → MAVLink Controller (mavlink_controller.py: request_takeoff)
  → ArduPilot Flight Controller
```

---

## Altitude Display in Frontend

### Telemetry Panel Shows:

1. **Altitude**: Absolute altitude (MSL) from `GLOBAL_POSITION_INT.altitude`
2. **Relative Alt**: Height above home position from `GLOBAL_POSITION_INT.relative_alt`
3. **TOF Down**: Distance to ground from downward TOF sensor (if available)

**Location**: `frontend/src/components/TelemetryPanel.jsx`

```jsx
<p><strong>Altitude:</strong> {telemetry.altitude ?? '—'} m</p>
<p><strong>Relative Alt:</strong> {telemetry.relative_alt ?? '—'} m</p>
<p><strong>TOF Down:</strong> {telemetry.tof_down ?? '—'} m</p>
```

---

## Safety Considerations

### Maximum Altitude

**Geofence**: Configurable in `production.yaml`:
```yaml
safety:
  geofence:
    enabled: false
    max_altitude: 50  # Maximum altitude in meters
```

### Tracking Altitude Limits

**Tracking Controller**: Limits altitude adjustment during tracking:
```yaml
tracking:
  altitude:
    auto_adjust: true
    min_height: 1.0   # Minimum tracking height
    max_height: 10.0  # Maximum tracking height
```

---

## Testing Recommendations

### Verify Altitude Measurement:

1. **Ground Test**: Before flight, verify `relative_alt` reads ~0m when on ground
2. **Takeoff Test**: After takeoff, verify `relative_alt` matches `takeoff_altitude` (2.0m)
3. **TOF Cross-Check**: Compare `tof_down` with `relative_alt` (should be similar at low altitude)
4. **Climb Test**: Manually climb and verify altitude updates smoothly

### Common Issues:

1. **Zero Altitude**: If altitude reads 0, check:
   - MAVLink connection
   - GPS lock (needed for accurate altitude)
   - Barometer calibration

2. **Incorrect Altitude**: If altitude seems wrong:
   - Check home position was set correctly
   - Verify barometer calibration
   - Check for GPS errors

3. **TOF Not Reading**: If `tof_down` is null:
   - TOF sensor is optional (system works without it)
   - Check TOF sensor wiring
   - Verify sensor initialization

---

## Technical Details

### MAVLink Message Format

```
GLOBAL_POSITION_INT:
  time_boot_ms: uint32_t
  lat: int32_t (degrees * 1e7)
  lon: int32_t (degrees * 1e7)
  alt: int32_t (mm, MSL altitude)
  relative_alt: int32_t (mm, relative to home)
  vx: int16_t (cm/s)
  vy: int16_t (cm/s)
  vz: int16_t (cm/s)
  hdg: uint16_t (cdeg)
```

### Conversion Factors

- **Latitude/Longitude**: Divide by `1e7` to get degrees
- **Altitude**: Divide by `1000.0` to get meters
- **Velocity**: Divide by `100.0` to get m/s
- **Heading**: Divide by `100.0` to get degrees

---

*Last Updated: 2024*
*Based on: ArduPilot MAVLink Protocol*

