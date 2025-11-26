#!/usr/bin/env python3
"""
Simple takeoff-hold-land test using downward TOF sensor for altitude.
- Assumes drone is in STABILIZE mode.
- Uses rc_override to control throttle incrementally.
- Does NOT use GUIDED mode (since no GPS).
- Uses only downward TOF sensor on multiplexer channel 1.
- Safe for bench testing with props REMOVED or disarmed (see notes).
"""

import time
import signal
import sys
import errno
import board
import busio
from smbus2 import SMBus
import adafruit_vl53l1x
from pymavlink import mavutil

# === Configuration ===
TARGET_ALTITUDE = 1.5  # meters
HOLD_DURATION = 60     # seconds
THROTTLE_STEP = 50     # PWM step size (e.g., 1100 -> 1150)
THROTTLE_MIN = 1000    # Minimum PWM (disarmed/idle)
THROTTLE_MAX = 2000    # Max PWM (full throttle)

# TOF Sensor
MULTIPLEXER_ADDR = 0x70
TOF_CHANNEL = 1        # Downward sensor
VL53L1X_ADDR = 0x29

# I2C retry config
MAX_I2C_RETRIES = 3

# === Global state ===
running = True

def signal_handler(sig, frame):
    global running
    print("\n🛑 Interrupt received. Landing immediately.")
    running = False

signal.signal(signal.SIGINT, signal_handler)

def select_mux_channel(bus, channel):
    try:
        bus.write_byte(MULTIPLEXER_ADDR, 1 << channel)
        time.sleep(0.01)
    except OSError as e:
        if e.errno == errno.EAGAIN or e.errno == 11:
            time.sleep(0.02)
            bus.write_byte(MULTIPLEXER_ADDR, 1 << channel)
            time.sleep(0.01)
        else:
            raise

def read_tof_distance(smbus, tof_sensor):
    try:
        select_mux_channel(smbus, TOF_CHANNEL)
        time.sleep(0.01)
        if tof_sensor.data_ready:
            dist_cm = tof_sensor.distance
            tof_sensor.clear_interrupt()
            return dist_cm / 100.0 if dist_cm is not None else None  # meters
    except (OSError, ValueError):
        pass
    return None

def send_rc_throttle(master, throttle_pwm):
    """Send RC override with throttle only (others neutral)."""
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        1500,  # Roll
        1500,  # Pitch
        throttle_pwm,  # Throttle
        1500,  # Yaw
        0, 0, 0, 0
    )

def disarm(master):
    print("Disarming...")
    master.arducopter_disarm()
    time.sleep(1)

def main():
    global running

    print("🔌 Connecting to flight controller...")
    master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=256000)
    master.wait_heartbeat(timeout=10)
    print("✅ Heartbeat received.")

    # Safety: Ensure we are in STABILIZE mode (MODE 6 for ArduCopter)
    # You can change this if using a different mode that allows manual throttle
    print("✈️  Setting STABILIZE mode (mode 6)...")
    master.set_mode(6)  # STABILIZE
    time.sleep(1)

    # Initialize I2C and TOF
    print("📡 Initializing I2C and TOF sensor...")
    smbus = SMBus(1)
    i2c = busio.I2C(board.SCL, board.SDA, frequency=100000)
    select_mux_channel(smbus, TOF_CHANNEL)
    time.sleep(0.1)
    tof = adafruit_vl53l1x.VL53L1X(i2c, address=VL53L1X_ADDR)
    tof.start_ranging()
    print("✅ TOF sensor ready.")

    current_throttle = THROTTLE_MIN
    altitude = 0.0

    try:
        # Wait for stable ground reading
        print("📏 Calibrating ground distance...")
        ground_readings = []
        for _ in range(20):
            d = read_tof_distance(smbus, tof)
            if d is not None:
                ground_readings.append(d)
            time.sleep(0.05)
        if not ground_readings:
            raise RuntimeError("Failed to read TOF sensor")
        ground_alt = sum(ground_readings) / len(ground_readings)
        print(f"🟢 Ground reference: {ground_alt:.3f} m")

        # === ARMING ===
        print("⚠️  Ensure props are OFF or drone is secured!")
        print("Arming in 3 seconds...")
        time.sleep(3)
        master.arducopter_arm()
        print("✅ Armed!")

        # === Takeoff Phase ===
        print(f"🛫 Taking off to {TARGET_ALTITUDE}m using TOF...")
        while running and altitude < TARGET_ALTITUDE - 0.1:
            altitude_raw = read_tof_distance(smbus, tof)
            if altitude_raw is not None:
                altitude = altitude_raw - ground_alt
                if altitude < 0:
                    altitude = 0
            else:
                print("⚠️  TOF read failed – holding throttle")
            
            if current_throttle < THROTTLE_MAX:
                current_throttle += THROTTLE_STEP
            send_rc_throttle(master, current_throttle)
            print(f"Alt: {altitude:.2f}m | Throttle: {current_throttle}")
            time.sleep(0.2)

        # Stabilize at target
        print("✅ Target altitude reached. Holding...")

        # Hold for 60 seconds
        hold_start = time.time()
        while running and (time.time() - hold_start) < HOLD_DURATION:
            altitude_raw = read_tof_distance(smbus, tof)
            if altitude_raw is not None:
                altitude = altitude_raw - ground_alt
                if altitude < 0:
                    altitude = 0
            # Optional: fine-tune throttle based on error (simple P control)
            error = TARGET_ALTITUDE - altitude
            adj = int(error * 150)  # crude proportional gain
            throttle_cmd = min(THROTTLE_MAX, max(THROTTLE_MIN, current_throttle + adj))
            send_rc_throttle(master, throttle_cmd)
            print(f"Hovering: {altitude:.2f}m (throttle={throttle_cmd})")
            time.sleep(0.2)

        # === Landing Phase ===
        print("🛬 Landing...")
        while running and altitude > 0.05:
            altitude_raw = read_tof_distance(smbus, tof)
            if altitude_raw is not None:
                altitude = altitude_raw - ground_alt
                if altitude < 0:
                    altitude = 0
            current_throttle = max(THROTTLE_MIN, current_throttle - THROTTLE_STEP // 2)
            send_rc_throttle(master, current_throttle)
            print(f"Landing: {altitude:.2f}m | Throttle: {current_throttle}")
            time.sleep(0.2)

        # Cut throttle
        send_rc_throttle(master, THROTTLE_MIN)
        time.sleep(1)

    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        print("🪂 Final throttle cut & disarm")
        send_rc_throttle(master, THROTTLE_MIN)
        time.sleep(0.5)
        disarm(master)
        tof.stop_ranging()
        smbus.close()
        i2c.deinit()
        print("✅ Done.")

if __name__ == "__main__":
    main()