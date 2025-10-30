#!/usr/bin/env python3
"""
Basic arm/disarm test - updated with improved connection handling
Based on simple_heartbeat_test.py improvements
"""

from pymavlink import mavutil
import time
import sys

# Try common configurations (in order of preference)
CONFIGS = [
    ('/dev/ttyAMA0', 256000),
    ('/dev/ttyAMA0', 115200),
    ('/dev/ttyAMA10', 256000),
    ('/dev/ttyAMA10', 115200),
    ('/dev/serial0', 256000),
    ('/dev/serial0', 115200),
]

def connect_to_fc():
    """Connect to flight controller, trying different ports/baud rates."""
    for port, baud in CONFIGS:
        try:
            print(f"Trying {port} @ {baud} baud...")
            master = mavutil.mavlink_connection(port, baud=baud)
            
            print("Waiting for heartbeat...")
            master.wait_heartbeat(timeout=10)
            print("✓ Heartbeat received!")
            
            # Read heartbeat to verify connection and get system info
            fresh_hb = None
            start = time.time()
            while fresh_hb is None and (time.time() - start) < 3:
                msg = master.recv_match(type='HEARTBEAT', blocking=False)
                if msg:
                    if msg.get_srcComponent() == 1:  # Prefer autopilot
                        fresh_hb = msg
                        break
                    elif fresh_hb is None:
                        fresh_hb = msg
                time.sleep(0.1)
            
            if fresh_hb:
                system_id = fresh_hb.get_srcSystem()
                component_id = fresh_hb.get_srcComponent()
                print(f"✓ Connected! System ID: {system_id}, Component ID: {component_id}")
                
                # Check vehicle type
                if hasattr(fresh_hb, 'type'):
                    vehicle_types = {
                        1: "Generic", 2: "Fixed Wing", 3: "Quadrotor",
                        4: "Coaxial Helicopter", 5: "Normal Helicopter",
                        13: "Hexarotor", 14: "Octorotor"
                    }
                    vtype = vehicle_types.get(fresh_hb.type, f"Unknown ({fresh_hb.type})")
                    print(f"✓ Vehicle Type: {vtype}")
            
            return master
            
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
    
    print("\n❌ Could not connect to flight controller!")
    print("Troubleshooting:")
    print("1. Check serial console is disabled: sudo raspi-config")
    print("2. Verify enable_uart=1 in /boot/config.txt")
    print("3. Add user to dialout: sudo usermod -a -G dialout $USER")
    print("4. Check physical connections (TX/RX/GND)")
    print("5. Verify flight controller is powered on")
    sys.exit(1)

# Connect to flight controller
print("=" * 50)
print("Basic Arm/Disarm Test")
print("=" * 50)
master = connect_to_fc()

try:
    # Check current armed status
    try:
        hb = master.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
        if hb:
            is_armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            print(f"\nCurrent status: {'ARMED' if is_armed else 'DISARMED'}")
    except:
        pass
    
    # Arm the drone
    print("\nArming...")
    master.arducopter_arm()
    
    # Wait a moment and verify armed status
    time.sleep(1)
    try:
        hb = master.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
        if hb:
            is_armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            if is_armed:
                print("✓ Armed successfully!")
            else:
                print("⚠️  Arming command sent, but status shows DISARMED")
                print("   Check pre-arm checks (GPS, battery, etc.)")
    except:
        print("⚠️  Could not verify armed status")
    
    # Wait 5 seconds
    print("\nWaiting 5 seconds (drone is armed)...")
    time.sleep(5)
    
    # Disarm the drone
    print("\nDisarming...")
    master.arducopter_disarm()
    
    # Wait a moment and verify disarmed status
    time.sleep(1)
    try:
        hb = master.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
        if hb:
            is_armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            if not is_armed:
                print("✓ Disarmed successfully!")
            else:
                print("⚠️  Disarm command sent, but status shows ARMED")
    except:
        print("⚠️  Could not verify disarmed status")
    
    print("\n✓ Done.")
    
except KeyboardInterrupt:
    print("\n\n⚠️  Interrupted by user!")
    print("Attempting to disarm before exit...")
    try:
        master.arducopter_disarm()
        print("✓ Disarm command sent")
    except:
        pass
    sys.exit(1)
except Exception as e:
    print(f"\n❌ Error during arm/disarm test: {e}")
    print("Attempting to disarm before exit...")
    try:
        master.arducopter_disarm()
    except:
        pass
    sys.exit(1)
finally:
    master.close()
    print("Connection closed.")
