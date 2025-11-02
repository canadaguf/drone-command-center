#!/usr/bin/env python3
"""
Check current arming parameters and explain the difference between RC and MAVLink arming.
"""

import time
from pymavlink import mavutil

# Configuration
CONNECTION_STRING = "/dev/ttyAMA0"
BAUD = 256000

print("\n" + "=" * 60)
print("Arming Parameters Check")
print("=" * 60 + "\n")

# Connect
print(f"Connecting to {CONNECTION_STRING} @ {BAUD} baud...")
master = mavutil.mavlink_connection(CONNECTION_STRING, baud=BAUD)

# Wait for heartbeat
print("Waiting for heartbeat...")
master.wait_heartbeat()
print(f"✓ Connected!\n")

def get_parameter(param_name):
    """Get a parameter value."""
    master.mav.param_request_read_send(
        master.target_system,
        master.target_component,
        param_name.encode('utf-8'),
        -1
    )
    
    msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=5)
    if msg and msg.param_id.decode('utf-8').strip('\x00') == param_name:
        return msg.param_value
    return None

print("Checking arming-related parameters...\n")

# Check ARMING_CHECK
arming_check = get_parameter("ARMING_CHECK")
if arming_check is not None:
    print(f"ARMING_CHECK = {int(arming_check)}")
    
    # Decode bitmask
    ac = int(arming_check)
    if ac == 1:
        print("  → All pre-arm checks enabled")
    elif ac == 0:
        print("  → ALL checks DISABLED (unsafe!)")
    else:
        print("  → Custom checks enabled:")
        checks = {
            1: "All",
            2: "Barometer",
            4: "Compass",
            8: "GPS",
            16: "INS (IMU)",
            32: "Parameters",
            64: "RC Channels",
            128: "Board Voltage",
            256: "Battery Level",
            1024: "Logging",
            2048: "Safety Switch",
            4096: "GPS Config"
        }
        for bit, name in checks.items():
            if ac & bit:
                print(f"    - {name}")
    
    # Check if compass check is enabled
    if ac & 4 or ac == 1:
        print("\n⚠️  COMPASS CHECK IS ENABLED")
        print("   This is why MAVLink arming fails!")
else:
    print("Could not read ARMING_CHECK parameter")

# Check compass-related parameters
print("\n" + "-" * 60)
print("Compass settings:")
print("-" * 60)

use_compass = get_parameter("COMPASS_USE")
if use_compass is not None:
    print(f"COMPASS_USE = {int(use_compass)} ({'Enabled' if use_compass else 'Disabled'})")

auto_dec = get_parameter("COMPASS_AUTODEC")
if auto_dec is not None:
    print(f"COMPASS_AUTODEC = {int(auto_dec)} ({'Enabled' if auto_dec else 'Disabled'})")

# Check flight mode
print("\n" + "-" * 60)
print("Current status:")
print("-" * 60)

hb = master.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
if hb:
    is_armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
    print(f"Armed: {is_armed}")
    print(f"Flight mode: {master.flightmode}")

print("\n" + "=" * 60)
print("EXPLANATION: Why RC works but MAVLink doesn't")
print("=" * 60)
print("""
When you arm via RC, ArduPilot may be more permissive with certain checks
depending on your configuration. MAVLink arming is typically stricter.

You have 3 options:

OPTION 1 (Recommended for testing): Disable compass check only
  Run: python fix_compass_arming.py
  This keeps most safety checks but removes compass requirement

OPTION 2 (Quick fix): Calibrate the compass
  - Use Mission Planner or QGroundControl
  - Do compass calibration dance (rotate in all directions)
  - This is the "proper" fix but requires GCS software

OPTION 3 (Aggressive): Disable ALL checks
  Set ARMING_CHECK = 0
  ⚠️  DANGEROUS: Only for bench testing!

For now, I recommend OPTION 1 - it's safe for testing.
""")

master.close()

