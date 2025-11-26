#!/usr/bin/env python3
"""
Disable compass usage - for indoor testing without GPS/compass module.
"""

import time
from pymavlink import mavutil

CONNECTION_STRING = "/dev/ttyAMA0"
BAUD = 256000

print("\n" + "=" * 60)
print("Disable Compass")
print("=" * 60 + "\n")

# Connect
print(f"Connecting to {CONNECTION_STRING} @ {BAUD} baud...")
master = mavutil.mavlink_connection(CONNECTION_STRING, baud=BAUD)

print("Waiting for heartbeat...")
master.wait_heartbeat()
print(f"✓ Connected!\n")

def set_parameter(param_name, param_value):
    """Set a parameter value."""
    print(f"Setting {param_name} = {param_value}...")
    
    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        param_name.encode('utf-8'),
        param_value,
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32
    )
    
    # Wait for acknowledgment
    msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=5)
    if msg:
        # Handle both string and bytes for param_id
        param_id = msg.param_id if isinstance(msg.param_id, str) else msg.param_id.decode('utf-8')
        param_id = param_id.strip('\x00')
        if param_id == param_name:
            print(f"  ✓ {param_name} set to {msg.param_value}")
            return True
    
    print(f"  ✗ Failed to set {param_name}")
    return False

print("Disabling compass usage...\n")

# Disable compass - this tells ArduPilot not to use compass at all
set_parameter("COMPASS_USE", 0)

print("\n" + "=" * 60)
print("SUCCESS!")
print("=" * 60)
print("\nCompass has been disabled.")
print("You can now safely disconnect the GPS/compass module.")
print("\nNext steps:")
print("1. Power cycle the flight controller")
print("2. Try arming with: python worked_best/basic_arm.py")
print("\nNote: Without compass, the drone will drift in yaw (rotation).")
print("This is fine for indoor bench testing!")
print("=" * 60 + "\n")

time.sleep(2)
master.close()

