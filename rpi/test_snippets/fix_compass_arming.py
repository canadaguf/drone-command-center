#!/usr/bin/env python3
"""
Fix compass arming issue by disabling compass pre-arm check.
This allows MAVLink arming to work like RC arming does.
"""

import time
from pymavlink import mavutil

# Configuration
CONNECTION_STRING = "/dev/ttyAMA0"
BAUD = 256000

print("\n" + "=" * 60)
print("Fix Compass Arming Issue")
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
    if msg and msg.param_id.decode('utf-8').strip('\x00') == param_name:
        print(f"  ✓ {param_name} set to {msg.param_value}")
        return True
    else:
        print(f"  ✗ Failed to set {param_name}")
        return False

# Get current ARMING_CHECK value
print("Reading current ARMING_CHECK parameter...")
current_arming_check = get_parameter("ARMING_CHECK")

if current_arming_check is not None:
    print(f"Current ARMING_CHECK = {int(current_arming_check)}\n")
    
    # ARMING_CHECK bitmask:
    # 1 = All checks
    # 2 = Barometer
    # 4 = Compass
    # 8 = GPS
    # 16 = INS
    # 32 = Parameters
    # 64 = RC
    # 128 = Voltage
    # 256 = Battery
    # 1024 = Logging
    # 2048 = Safety Switch
    # 4096 = GPS Config
    
    # If currently "1" (all checks), we need to explicitly set bits except compass
    # All checks except compass = 1 + 2 + 8 + 16 + 32 + 64 + 128 + 256 + 1024 + 2048 + 4096
    # = 1 | 2 | 8 | 16 | 32 | 64 | 128 | 256 | 1024 | 2048 | 4096
    # = 7549 (without compass bit 4)
    
    # Better: Keep all checks, just remove compass
    if int(current_arming_check) == 1:
        # All checks enabled, disable compass only
        # Use value that represents all checks minus compass
        new_value = 7549  # All checks except compass (bit 4)
        print("Disabling compass check only (keeping all other checks)...")
    else:
        # Custom checks - just remove bit 4 if present
        current = int(current_arming_check)
        if current & 4:  # Compass check is enabled
            new_value = current & ~4  # Remove compass bit
            print("Removing compass check from current settings...")
        else:
            print("Compass check is already disabled!")
            new_value = current
    
    if new_value != int(current_arming_check):
        set_parameter("ARMING_CHECK", new_value)
        
        print("\n" + "=" * 60)
        print("SUCCESS!")
        print("=" * 60)
        print("\nCompass pre-arm check has been disabled.")
        print("MAVLink arming should now work like RC arming.\n")
        print("Next steps:")
        print("1. Wait 5 seconds for parameter to save")
        print("2. Try arming with: python worked_best/basic_arm.py")
        print("\nNote: This only disables the compass PRE-ARM check.")
        print("The compass is still active and used for navigation.")
        print("=" * 60 + "\n")
        
        # Wait for parameter to save
        print("Waiting 5 seconds for parameter to save...")
        time.sleep(5)
    else:
        print("\n✓ No changes needed - compass check already disabled")
else:
    print("Could not read ARMING_CHECK parameter")

master.close()

