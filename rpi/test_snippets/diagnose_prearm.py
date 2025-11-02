#!/usr/bin/env python3
"""
Diagnostic script to check pre-arm failures.
Reads STATUSTEXT messages to see why arming is blocked.
"""

import time
from pymavlink import mavutil

# Configuration
CONNECTION_STRING = "/dev/ttyAMA0"
BAUD = 256000

print("\n" + "=" * 60)
print("Pre-Arm Diagnostics")
print("=" * 60 + "\n")

# Connect
print(f"Connecting to {CONNECTION_STRING} @ {BAUD} baud...")
master = mavutil.mavlink_connection(CONNECTION_STRING, baud=BAUD)

# Wait for heartbeat
print("Waiting for heartbeat...")
master.wait_heartbeat()
print(f"✓ Connected! System {master.target_system}, Component {master.target_component}\n")

# Request all parameters (optional, helps trigger messages)
print("Requesting parameter list...")
master.mav.param_request_list_send(
    master.target_system,
    master.target_component
)

# Try to arm and capture error messages
print("\n" + "=" * 60)
print("Attempting to ARM - watching for error messages...")
print("=" * 60 + "\n")

# Listen for status messages for 2 seconds first
print("Listening for existing status messages...")
start_time = time.time()
while time.time() - start_time < 2:
    msg = master.recv_match(type='STATUSTEXT', blocking=False, timeout=0.1)
    if msg:
        severity = msg.severity
        text = msg.text
        severity_map = {
            0: "EMERGENCY",
            1: "ALERT",
            2: "CRITICAL",
            3: "ERROR",
            4: "WARNING",
            5: "NOTICE",
            6: "INFO",
            7: "DEBUG"
        }
        severity_str = severity_map.get(severity, str(severity))
        print(f"[{severity_str}] {text}")

# Now try to arm
print("\nSending ARM command...")
master.arducopter_arm()

# Listen for response messages
print("\nWaiting for response (10 seconds)...")
start_time = time.time()
armed = False

while time.time() - start_time < 10:
    # Check for status text messages
    msg = master.recv_match(type='STATUSTEXT', blocking=False, timeout=0.1)
    if msg:
        severity = msg.severity
        text = msg.text
        severity_map = {
            0: "EMERGENCY",
            1: "ALERT", 
            2: "CRITICAL",
            3: "ERROR",
            4: "WARNING",
            5: "NOTICE",
            6: "INFO",
            7: "DEBUG"
        }
        severity_str = severity_map.get(severity, str(severity))
        print(f"[{severity_str}] {text}")
        
        if "PreArm" in text or "Arm" in text:
            print(f"  → PRE-ARM CHECK: {text}")
    
    # Check heartbeat for armed status
    hb = master.recv_match(type='HEARTBEAT', blocking=False, timeout=0.1)
    if hb:
        from pymavlink.dialects.v20 import ardupilotmega as mavlink2
        is_armed = bool(hb.base_mode & mavlink2.MAV_MODE_FLAG_SAFETY_ARMED)
        if is_armed and not armed:
            print("\n✓ ARMED SUCCESSFULLY!")
            armed = True
            break

if not armed:
    print("\n⚠️  Failed to arm")
    print("\nCommon pre-arm check failures:")
    print("  1. GPS: No GPS lock (need 3D fix with 6+ satellites)")
    print("  2. EKF: Navigation estimator not ready") 
    print("  3. RC: No RC calibration or RC failsafe active")
    print("  4. Battery: Voltage too low")
    print("  5. Compass: Not calibrated or inconsistent")
    print("  6. Accelerometer: Not calibrated")
    print("  7. Mode: Can't arm in certain modes (e.g., CIRCLE, AUTO without mission)")
    print("  8. Safety: Safety switch not pressed (if enabled)")
    print("\nTo disable GPS requirement for indoor testing:")
    print("  Set parameter: ARMING_CHECK = 0 (disables ALL checks - USE WITH CAUTION)")
    print("  Or: ARMING_CHECK = 388598 (disables GPS only)")

print("\n" + "=" * 60)
print("Diagnostic complete")
print("=" * 60 + "\n")

master.close()

