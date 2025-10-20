from pymavlink import mavutil

# Connect to the flight controller via UART
master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=115200)

# Wait for the first heartbeat
master.wait_heartbeat()
print("Heartbeat received! Drone is alive.")
