from pymavlink import mavutil
import time

# Connect to the flight controller
master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=256000)

# Wait for the first heartbeat
print("Waiting for heartbeat...")
master.wait_heartbeat()
print("Heartbeat received.")

# Arm the drone
print("Arming...")
master.arducopter_arm()

# Wait 5 seconds
time.sleep(5)

# Disarm the drone
print("Disarming...")
master.arducopter_disarm()

print("Done.")
