#!/usr/bin/env python3
"""
Comprehensive connection test for Raspberry Pi to Drone
Based on working snippets from worked_best/
Tests serial connection, heartbeat, and basic communication
"""

import os
import sys
import time
from pathlib import Path

try:
    from pymavlink import mavutil
except ImportError:
    print("ERROR: pymavlink not installed. Install with: pip install pymavlink")
    sys.exit(1)

# Test configuration - try both common baud rates
BAUD_RATES = [256000, 115200]
SERIAL_PATHS = ['/dev/ttyAMA0', '/dev/ttyAMA10', '/dev/serial0']

def check_serial_port(port_path):
    """Check if serial port exists and is accessible."""
    print(f"\n{'='*60}")
    print(f"Checking serial port: {port_path}")
    print(f"{'='*60}")
    
    if not os.path.exists(port_path):
        print(f"  ❌ Port {port_path} does not exist")
        return False
    
    print(f"  ✓ Port {port_path} exists")
    
    # Check permissions
    if not os.access(port_path, os.R_OK | os.W_OK):
        print(f"  ⚠️  Warning: No read/write permissions on {port_path}")
        print(f"     Fix with: sudo usermod -a -G dialout $USER")
        print(f"     Then logout and login again")
        return False
    
    print(f"  ✓ Port {port_path} is accessible")
    return True

def test_connection(port_path, baud_rate, timeout=10):
    """Test MAVLink connection with given port and baud rate."""
    print(f"\n{'='*60}")
    print(f"Testing connection: {port_path} @ {baud_rate} baud")
    print(f"{'='*60}")
    
    try:
        print(f"  Attempting connection...")
        master = mavutil.mavlink_connection(port_path, baud=baud_rate)
        
        print(f"  Waiting for heartbeat (timeout: {timeout}s)...")
        start_time = time.time()
        
        try:
            master.wait_heartbeat(timeout=timeout)
            elapsed = time.time() - start_time
            print(f"  ✓ Heartbeat received! (took {elapsed:.2f}s)")
            
            # Get fresh heartbeat with details
            print(f"  Getting heartbeat details...")
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
            
            if fresh_hb is None:
                print(f"  ⚠️  Could not get fresh heartbeat, trying blocking read")
                fresh_hb = master.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
            
            if fresh_hb:
                # Extract IDs from the heartbeat message itself
                system_id = fresh_hb.get_srcSystem()
                component_id = fresh_hb.get_srcComponent()
                
                print(f"\n  📊 Heartbeat Details:")
                print(f"     System ID: {system_id}")
                print(f"     Component ID: {component_id}")
                
                # Vehicle type
                vehicle_types = {
                    1: "Generic",
                    2: "Fixed Wing",
                    3: "Quadrotor",
                    4: "Coaxial Helicopter",
                    5: "Normal Helicopter",
                    13: "Hexarotor",
                    14: "Octorotor"
                }
                vtype = vehicle_types.get(fresh_hb.type, f"Unknown ({fresh_hb.type})")
                print(f"     Vehicle Type: {vtype}")
                
                # Autopilot type
                autopilot_types = {
                    3: "ArduPilot",
                    4: "PX4",
                    8: "PX4"
                }
                ap_type = autopilot_types.get(fresh_hb.autopilot, f"Unknown ({fresh_hb.autopilot})")
                print(f"     Autopilot: {ap_type}")
                
                # Armed status
                is_armed = bool(fresh_hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                is_manual = bool(fresh_hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED)
                print(f"     Armed: {'YES ✓' if is_armed else 'NO ✗'}")
                print(f"     Manual Control: {'YES ✓' if is_manual else 'NO ✗'}")
                
                # Flight mode
                try:
                    mode = master.flightmode
                    print(f"     Flight Mode: {mode}")
                except:
                    print(f"     Flight Mode: Unknown")
                
                master.close()
                return True
            else:
                print(f"  ❌ Could not read heartbeat details")
                master.close()
                return False
                
        except Exception as e:
            elapsed = time.time() - start_time
            print(f"  ❌ No heartbeat received (waited {elapsed:.2f}s)")
            print(f"     Error: {e}")
            master.close()
            return False
            
    except Exception as e:
        print(f"  ❌ Connection failed: {e}")
        return False

def main():
    """Main test function."""
    print("\n" + "="*60)
    print("Raspberry Pi Drone Connection Test")
    print("="*60)
    print("\nThis script tests the connection to your drone's flight controller.")
    print("It will try different serial ports and baud rates.")
    
    # Check Python version
    print(f"\nPython version: {sys.version}")
    
    # Check pymavlink version
    try:
        import pymavlink
        print(f"pymavlink version: {pymavlink.__version__}")
    except:
        print("pymavlink version: Unknown")
    
    # Check if user is in dialout group
    try:
        import grp
        groups = [g.gr_name for g in grp.getgrall() if os.getlogin() in g.gr_mem]
        if 'dialout' in groups:
            print("✓ User is in 'dialout' group")
        else:
            print("⚠️  User is NOT in 'dialout' group")
            print("   Add yourself: sudo usermod -a -G dialout $USER")
            print("   Then logout and login again")
    except:
        pass
    
    # Test each serial port
    working_config = None
    
    for port_path in SERIAL_PATHS:
        if not check_serial_port(port_path):
            continue
        
        for baud_rate in BAUD_RATES:
            if test_connection(port_path, baud_rate):
                working_config = (port_path, baud_rate)
                print(f"\n{'='*60}")
                print(f"✓ SUCCESS! Working configuration found:")
                print(f"  Port: {port_path}")
                print(f"  Baud Rate: {baud_rate}")
                print(f"{'='*60}\n")
                break
        
        if working_config:
            break
    
    if not working_config:
        print(f"\n{'='*60}")
        print("❌ CONNECTION FAILED")
        print(f"{'='*60}")
        print("\nTroubleshooting steps:")
        print("1. Check physical connections (UART TX/RX/GND)")
        print("2. Verify serial console is disabled:")
        print("   - Run: sudo raspi-config")
        print("   - Go to: Interface Options → Serial Port")
        print("   - Disable: 'Serial login shell'")
        print("   - Enable: 'Serial interface'")
        print("   - Reboot")
        print("3. Check /boot/config.txt:")
        print("   - Should have: enable_uart=1")
        print("   - Should NOT have: console=serial0,115200")
        print("4. Verify you're in dialout group:")
        print("   - Run: groups")
        print("   - Should see 'dialout' in the list")
        print("5. Check if flight controller is powered on")
        print("6. Try different baud rates (check flight controller settings)")
        print("\n")
        sys.exit(1)
    else:
        print("✓ Connection test PASSED!")
        print(f"\nRecommended configuration:")
        print(f"  Connection: {working_config[0]}")
        print(f"  Baud Rate: {working_config[1]}")
        print("\nYou can now use this configuration in your drone client.")
        sys.exit(0)

if __name__ == "__main__":
    main()


