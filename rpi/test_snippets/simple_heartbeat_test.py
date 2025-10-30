#!/usr/bin/env python3
"""
Simple heartbeat test - based on worked_best/heart_check.py
Minimal test to verify basic connection and heartbeat
"""

from pymavlink import mavutil
import sys
import time

# Try both common configurations
CONFIGS = [
    ('/dev/ttyAMA0', 256000),
    ('/dev/ttyAMA0', 115200),
    ('/dev/ttyAMA10', 256000),
    ('/dev/ttyAMA10', 115200),
    ('/dev/serial0', 256000),
    ('/dev/serial0', 115200),
]

print("Testing MAVLink connection...")
print("=" * 50)

for port, baud in CONFIGS:
    try:
        print(f"\nTrying {port} @ {baud} baud...")
        master = mavutil.mavlink_connection(port, baud=baud)
        
        print("  Waiting for heartbeat...")
        master.wait_heartbeat(timeout=5)
        
        # Read a fresh heartbeat message to get proper system/component IDs
        print("  Reading heartbeat details...")
        fresh_hb = None
        start = time.time()
        while fresh_hb is None and (time.time() - start) < 3:
            msg = master.recv_match(type='HEARTBEAT', blocking=False)
            if msg:
                # Prefer autopilot heartbeat (component ID 1), but accept any
                if msg.get_srcComponent() == 1:
                    fresh_hb = msg
                    break
                elif fresh_hb is None:  # Store first heartbeat as fallback
                    fresh_hb = msg
            time.sleep(0.1)
        
        # If no heartbeat found in non-blocking mode, try blocking
        if fresh_hb is None:
            fresh_hb = master.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
        
        if fresh_hb is None:
            print(f"  ⚠️  Heartbeat received but couldn't read details")
            print(f"  ✓ Connection: {port}")
            print(f"  ✓ Baud Rate: {baud}")
            print(f"  ⚠️  System ID: {master.target_system} (may be 0)")
            print(f"  ⚠️  Component ID: {master.target_component} (may be 0)")
        else:
            # Extract IDs from the heartbeat message
            system_id = fresh_hb.get_srcSystem()
            component_id = fresh_hb.get_srcComponent()
            
            print(f"  ✓ SUCCESS! Heartbeat received!")
            print(f"  ✓ Connection: {port}")
            print(f"  ✓ Baud Rate: {baud}")
            print(f"  ✓ System ID: {system_id}")
            print(f"  ✓ Component ID: {component_id}")
            
            # Show additional info if available
            if hasattr(fresh_hb, 'type'):
                vehicle_types = {
                    1: "Generic", 2: "Fixed Wing", 3: "Quadrotor",
                    4: "Coaxial Helicopter", 5: "Normal Helicopter",
                    13: "Hexarotor", 14: "Octorotor"
                }
                vtype = vehicle_types.get(fresh_hb.type, f"Unknown ({fresh_hb.type})")
                print(f"  ✓ Vehicle Type: {vtype}")
            
            if hasattr(fresh_hb, 'autopilot'):
                ap_types = {3: "ArduPilot", 4: "PX4", 8: "PX4"}
                ap_type = ap_types.get(fresh_hb.autopilot, f"Unknown ({fresh_hb.autopilot})")
                print(f"  ✓ Autopilot: {ap_type}")
        
        master.close()
        sys.exit(0)
        
    except FileNotFoundError:
        print(f"  ✗ Port {port} not found")
        continue
    except PermissionError:
        print(f"  ✗ Permission denied on {port}")
        print(f"     Fix: sudo usermod -a -G dialout $USER")
        continue
    except Exception as e:
        print(f"  ✗ Failed: {e}")
        continue

print("\n" + "=" * 50)
print("❌ All connection attempts failed!")
print("\nTroubleshooting:")
print("1. Check serial console is disabled: sudo raspi-config")
print("2. Verify enable_uart=1 in /boot/config.txt")
print("3. Add user to dialout: sudo usermod -a -G dialout $USER")
print("4. Reboot after making changes")
sys.exit(1)


