from pymavlink import mavutil

master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=115200)
master.wait_heartbeat()

# Request GPS_RAW_INT
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
    0,
    mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT,
    0, 0, 0, 0, 0, 0
)

gps = master.recv_match(type='GPS_RAW_INT', blocking=True, timeout=5)
if gps:
    fix_type = gps.fix_type
    lat = gps.lat / 1e7  # degrees
    lon = gps.lon / 1e7  # degrees
    alt_msl = gps.alt / 1e3  # meters (MSL, per MAVLink spec)
    
    print(f"GPS Fix: {fix_type} (3 = 3D fix)")
    print(f"Lat: {lat:.6f}, Lon: {lon:.6f} (degrees)")
    print(f"Alt (MSL): {alt_msl:.1f} m")
    
    # Optional: ellipsoid altitude if available
    if hasattr(gps, 'alt_ellipsoid'):
        alt_ell = gps.alt_ellipsoid / 1e3
        print(f"Alt (ellipsoid): {alt_ell:.1f} m")
else:
    print("No GPS data (check antenna/sky view)")
