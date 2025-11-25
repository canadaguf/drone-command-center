from pymavlink import mavutil
import time

print("Connecting to flight controller...")
master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=256000)

print("Waiting for heartbeat...")
try:
    master.wait_heartbeat(timeout=10)
    print("Heartbeat received!")
except Exception as e:
    print(f"No heartbeat in 10 seconds: {e}")
    exit(1)

# Get a FRESH heartbeat to check current state
print("Requesting fresh heartbeat for up-to-date status...")
fresh_hb = None
start = time.time()
while fresh_hb is None and (time.time() - start) < 3:
    msg = master.recv_match(type='HEARTBEAT', blocking=False)
    if msg:
        # Skip heartbeats from GCS or companion (only care about autopilot)
        if msg.get_srcComponent() == 1:  # Typically autopilot = comp ID 1
            fresh_hb = msg
    time.sleep(0.1)

if fresh_hb is None:
    print("Could not get fresh heartbeat; using initial one.")
    fresh_hb = master.recv_match(type='HEARTBEAT', blocking=True, timeout=2)

if fresh_hb is None:
    print("Could not read heartbeat details.")
    exit(1)

# Validate vehicle type
if fresh_hb.type in (mavutil.mavlink.MAV_TYPE_QUADROTOR, mavutil.mavlink.MAV_TYPE_HEXAROTOR):
    print(f"Vehicle type: Copter (MAV_TYPE = {fresh_hb.type})")
else:
    print(f"Unexpected vehicle type: {fresh_hb.type}")

# Autopilot type
if fresh_hb.autopilot == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
    print("Autopilot: ArduPilot")
elif fresh_hb.autopilot == mavutil.mavlink.MAV_AUTOPILOT_PX4:
    print("Autopilot: PX4")
else:
    print(f"Unknown autopilot: {fresh_hb.autopilot}")

print(f"System ID: {master.target_system}, Component ID: {master.target_component}")

# CORRECT ARMED CHECK: Use fresh heartbeat
is_armed = bool(fresh_hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
is_manual = bool(fresh_hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED)

print(f"Armed: {'YES' if is_armed else 'NO'}")
print(f"Manual control enabled: {'YES' if is_manual else 'NO'}")

# Optional: Firmware version
try:
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
        0,
        mavutil.mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION,
        0, 0, 0, 0, 0, 0
    )
    ver = master.recv_match(type='AUTOPILOT_VERSION', blocking=True, timeout=2)
    if ver:
        fw = ver.flight_sw_version
        print(f"Firmware version: {fw}")
    else:
        print("Firmware version: Not available")
except Exception as e:
    print(f"Firmware version: Could not retrieve ({e})")

print("\nConnection verified! You're talking to your drone.")