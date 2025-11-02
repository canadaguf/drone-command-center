#!/usr/bin/env python3
"""
Configure drone for indoor testing (no GPS required).
This script modifies ArduPilot parameters to allow arming without GPS.

WARNING: This disables safety checks! Only use for indoor bench testing.
"""

import time
from pymavlink import mavutil

# Configuration
CONNECTION_STRING = "/dev/ttyAMA0"
BAUD = 256000

print("\n" + "=" * 60)
print("Indoor Arming Configuration")
print("WARNING: This disables GPS and some safety checks!")
print("=" * 60 + "\n")

# Connect
print(f"Connecting to {CONNECTION_STRING} @ {BAUD} baud...")
master = mavutil.mavlink_connection(CONNECTION_STRING, baud=BAUD)

# Wait for heartbeat
print("Waiting for heartbeat...")
master.wait_heartbeat()
print(f"✓ Connected! System {master.target_system}, Component {master.target_component}\n")

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

print("Configuring parameters for indoor testing...\n")

# Option 1: Disable GPS requirement only (safer)
print("OPTION 1: Disable GPS requirement (safer for testing)")
print("-" * 60)

# ARMING_CHECK bitmask:
# Bit 0: All (if 0, disables all checks)
# Bit 1: Barometer
# Bit 2: Compass
# Bit 3: GPS
# Bit 4: INS (accelerometers & gyros)
# Bit 5: Parameters
# Bit 6: RC Channels
# Bit 7: Board Voltage
# Bit 8: Battery Level
# Bit 9: Airspeed (for planes)
# Bit 10: LoggingAvailable
# Bit 11: Hardware Safety Switch
# Bit 12: GPS Configuration

# Default ARMING_CHECK = 1 (all checks)
# To disable GPS: 1 (all) - 8 (GPS bit) = value that excludes GPS
# Proper value: 388598 (all checks except GPS)

# For testing, we'll disable GPS requirement
# ARMING_CHECK = 388598 means: all checks EXCEPT GPS
set_parameter("ARMING_CHECK", 388598)

print("\n" + "=" * 60)
print("OPTION 2: Disable ALL checks (ONLY for bench testing)")
print("-" * 60)
print("Uncomment the line below if you want to disable ALL checks:")
print("  # set_parameter('ARMING_CHECK', 0)")
print("")
# Uncomment next line to disable ALL arming checks (not recommended)
# set_parameter("ARMING_CHECK", 0)

print("\n" + "=" * 60)
print("Additional indoor testing parameters")
print("-" * 60)

# Set EKF origin (for indoor testing without GPS)
# This helps the EKF (Extended Kalman Filter) initialize without GPS
set_parameter("EK3_SRC1_POSXY", 0)  # 0 = None (no GPS for position)
set_parameter("EK3_SRC1_VELXY", 0)  # 0 = None (no GPS for velocity)
set_parameter("EK3_SRC1_POSZ", 1)   # 1 = Baro (use barometer for altitude)

# Alternative: Use optical flow or external positioning
# set_parameter("EK3_SRC1_POSXY", 6)  # 6 = ExternalNav
# set_parameter("EK3_SRC1_VELXY", 6)  # 6 = ExternalNav

print("\n" + "=" * 60)
print("Configuration complete!")
print("-" * 60)
print("\nNext steps:")
print("1. Wait 5 seconds for parameters to save")
print("2. Reboot the flight controller (power cycle)")
print("3. Try arming again with basic_arm.py")
print("\nIf still can't arm, run diagnose_prearm.py to see specific errors")
print("=" * 60 + "\n")

# Wait a bit to ensure parameters are saved
print("Waiting 5 seconds for parameters to save...")
time.sleep(5)

master.close()

