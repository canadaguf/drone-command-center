#!/usr/bin/env python3
"""
DroneKit GUIDED liftoff script that runs entirely standalone.

Sequence:
1. Initialize bottom ToF sensor using the same multiplexer flow as tof_test_multiplexer.py
2. Connect to ArduCopter over the provided serial link and switch to GUIDED
3. Arm motors, gently ramp throttle until ToF shows ~10cm, then hand over to simple_takeoff
4. Climb to 1m (configurable) with ToF confirmation, hold position for 60s, descend slowly
5. Touch down softly, cut throttle, disarm

Dry-run mode (`--dry-run`) skips DroneKit commands but still exercises the ToF logic/logging so
the script can be bench tested indoors without the aircraft powered.
"""

import argparse
import errno
import logging
import math
import signal
import sys
import time
from typing import Dict, Optional

from dronekit import VehicleMode, connect

import board
import busio
import adafruit_vl53l1x
from smbus2 import SMBus

# ---------------------------------------------------------------------------
# Configuration defaults
# ---------------------------------------------------------------------------

CONNECTION_STRING = "/dev/ttyAMA0"
BAUD_RATE = 256000
TARGET_ALTITUDE = 1.0  # meters
HOLD_DURATION = 60  # seconds
GROUND_THRESHOLD = 0.08  # meters
TAKEOFF_THRESHOLD = 0.1  # meters
ALTITUDE_TOLERANCE = 0.05  # meters
THROTTLE_IDLE = 1100
THROTTLE_MAX = 1900
THROTTLE_STEP = 4

MULTIPLEXER_ADDRESS = 0x70
VL53L1X_ADDRESS = 0x29
BOTTOM_SENSOR_CHANNEL = 1

MAX_I2C_RETRIES = 5
BASE_RETRY_DELAY = 0.02

logger = logging.getLogger("liftoff_guided")


# ---------------------------------------------------------------------------
# CLI / logging helpers
# ---------------------------------------------------------------------------

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Standalone GUIDED liftoff script with ToF altitude confirmation"
    )
    parser.add_argument("--connect", default=CONNECTION_STRING, help="MAVLink connection string")
    parser.add_argument("--baud", type=int, default=BAUD_RATE, help="Serial baud rate")
    parser.add_argument("--target-alt", type=float, default=TARGET_ALTITUDE, help="Target hover altitude in meters")
    parser.add_argument("--hold-seconds", type=int, default=HOLD_DURATION, help="Hover hold duration")
    parser.add_argument("--tof-channel", type=int, default=BOTTOM_SENSOR_CHANNEL, help="Multiplexer channel for bottom ToF")
    parser.add_argument("--dry-run", action="store_true", help="Skip DroneKit commands, only log planned actions")
    parser.add_argument("--log-level", default="INFO", help="Logging level (DEBUG, INFO, ...)")
    parser.add_argument("--max-throttle", type=int, default=THROTTLE_MAX, help="Upper throttle bound for ramp sequence")
    parser.add_argument("--idle-throttle", type=int, default=THROTTLE_IDLE, help="Initial throttle PWM for ramp sequence")
    parser.add_argument("--takeoff-threshold", type=float, default=TAKEOFF_THRESHOLD, help="Distance in meters that indicates liftoff")
    parser.add_argument("--ground-threshold", type=float, default=GROUND_THRESHOLD, help="Distance in meters treated as touchdown")
    return parser.parse_args()


def configure_logging(level: str) -> None:
    logging.basicConfig(
        level=getattr(logging, level.upper(), logging.INFO),
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )


# ---------------------------------------------------------------------------
# ToF helpers (based on rpi/tof_test_multiplexer.py)
# ---------------------------------------------------------------------------

def select_multiplexer_channel(bus: SMBus, channel: int, retry_count: int = 0) -> None:
    channel_mask = 1 << channel
    try:
        bus.write_byte(MULTIPLEXER_ADDRESS, channel_mask)
        time.sleep(0.01)
    except OSError as exc:
        if (exc.errno == errno.EAGAIN or exc.errno == 11) and retry_count < MAX_I2C_RETRIES:
            delay = BASE_RETRY_DELAY * (2 ** retry_count)
            logger.debug("Multiplexer busy, retrying channel %s in %.0fms", channel, delay * 1000)
            time.sleep(delay)
            select_multiplexer_channel(bus, channel, retry_count + 1)
        else:
            raise


def initialize_tof_sensor(channel: int) -> Dict[str, object]:
    logger.info("Initializing VL53L1X on multiplexer channel %s", channel)
    smbus = SMBus(1)
    i2c = busio.I2C(board.SCL, board.SDA, frequency=100000)
    select_multiplexer_channel(smbus, channel)
    time.sleep(0.1)

    sensor = adafruit_vl53l1x.VL53L1X(i2c, address=VL53L1X_ADDRESS)
    sensor.start_ranging()
    return {"smbus": smbus, "sensor": sensor, "channel": channel}


def read_tof_distance(sensor_bundle: Dict[str, object]) -> Optional[float]:
    sensor = sensor_bundle["sensor"]
    smbus = sensor_bundle["smbus"]
    channel = sensor_bundle["channel"]

    try:
        select_multiplexer_channel(smbus, channel)
        time.sleep(0.02)

        data_ready = False
        for attempt in range(3):
            try:
                data_ready = sensor.data_ready
                break
            except OSError as exc:
                if exc.errno == errno.EAGAIN or exc.errno == 11:
                    time.sleep(0.01 * (attempt + 1))
                    continue
                raise

        if not data_ready:
            return None

        distance_cm = None
        for attempt in range(3):
            try:
                distance_cm = sensor.distance
                break
            except OSError as exc:
                if exc.errno == errno.EAGAIN or exc.errno == 11:
                    time.sleep(0.01 * (attempt + 1))
                    continue
                raise

        if distance_cm is None:
            return None

        try:
            sensor.clear_interrupt()
        except OSError:
            pass

        distance_m = distance_cm / 100.0
        if distance_m < 0.0 or distance_m > 4.0:
            return None
        return distance_m
    except Exception as exc:
        logger.debug("ToF read error: %s", exc)
        return None


def shutdown_tof(sensor_bundle: Optional[Dict[str, object]]) -> None:
    if not sensor_bundle:
        return
    sensor = sensor_bundle.get("sensor")
    smbus = sensor_bundle.get("smbus")
    channel = sensor_bundle.get("channel")

    try:
        if sensor:
            if smbus is not None and channel is not None:
                select_multiplexer_channel(smbus, channel)
                time.sleep(0.01)
            sensor.stop_ranging()
    finally:
        if smbus:
            try:
                smbus.close()
            except Exception:
                pass


# ---------------------------------------------------------------------------
# DroneKit helpers
# ---------------------------------------------------------------------------

def wait_until_armable(vehicle, timeout: float, dry_run: bool) -> bool:
    if dry_run:
        logger.info("Dry-run: skipping armable wait")
        return True
    logger.info("Waiting for vehicle to become armable...")
    start = time.time()
    while time.time() - start < timeout:
        if vehicle.is_armable:
            logger.info("Vehicle is armable")
            return True
        time.sleep(1.0)
    logger.error("Vehicle never became armable")
    return False


def ensure_gps_lock(vehicle, min_fix: int, min_sat: int, dry_run: bool) -> bool:
    if dry_run:
        logger.info("Dry-run: skipping GPS lock check")
        return True
    gps = getattr(vehicle, "gps_0", None)
    if gps is None:
        logger.error("No GPS data available")
        return False
    logger.info("GPS fix_type=%s satellites=%s", gps.fix_type, gps.satellites_visible)
    if gps.fix_type >= min_fix and gps.satellites_visible >= min_sat:
        return True
    logger.error("Insufficient GPS lock (need fix>=%s sat>=%s)", min_fix, min_sat)
    return False


def set_mode(vehicle, mode: str, dry_run: bool, timeout: float = 5.0) -> bool:
    if dry_run:
        logger.info("[dry-run] Would set mode to %s", mode)
        return True
    logger.info("Setting mode to %s", mode)
    vehicle.mode = VehicleMode(mode)
    start = time.time()
    while time.time() - start < timeout:
        if vehicle.mode.name == mode:
            return True
        time.sleep(0.2)
    logger.error("Mode change to %s timed out", mode)
    return False


def arm_vehicle(vehicle, dry_run: bool, timeout: float = 10.0) -> bool:
    if dry_run:
        logger.info("[dry-run] Would arm vehicle")
        return True
    logger.info("Arming vehicle...")
    vehicle.armed = True
    start = time.time()
    while time.time() - start < timeout:
        if vehicle.armed:
            logger.info("Vehicle armed")
            return True
        time.sleep(0.2)
    logger.error("Arming timed out")
    return False


def apply_throttle_override(vehicle, pwm: int, dry_run: bool) -> None:
    pwm = max(1000, min(2000, pwm))
    if dry_run:
        logger.debug("[dry-run] Throttle override -> %s", pwm)
        return
    vehicle.channels.overrides = {
        "1": 1500,
        "2": 1500,
        "3": pwm,
        "4": 1500,
    }


def clear_rc_override(vehicle, dry_run: bool) -> None:
    if dry_run:
        logger.debug("[dry-run] Clear RC override")
        return
    vehicle.channels.overrides = {}


def gradual_liftoff(vehicle, tof_bundle, args, dry_run: bool) -> bool:
    logger.info("Ramping throttle until ToF exceeds %.2fm", args.takeoff_threshold)
    throttle = args.idle_throttle
    last_log = time.time()
    while throttle <= args.max_throttle:
        apply_throttle_override(vehicle, throttle, dry_run)
        time.sleep(0.15)
        distance = read_tof_distance(tof_bundle)
        if distance is not None and distance > args.takeoff_threshold:
            logger.info("Takeoff detected (%.2fm) at throttle %s", distance, throttle)
            clear_rc_override(vehicle, dry_run)
            return True
        throttle += THROTTLE_STEP
        if time.time() - last_log > 1.0:
            logger.debug("Throttle %s, ToF=%.3fm", throttle, distance if distance is not None else -1)
            last_log = time.time()
    logger.error("Reached max throttle without detecting liftoff")
    clear_rc_override(vehicle, dry_run)
    return False


def command_takeoff(vehicle, target_alt: float, dry_run: bool) -> bool:
    if dry_run:
        logger.info("[dry-run] Would call simple_takeoff(%.2f)", target_alt)
        return True
    logger.info("Calling simple_takeoff to %.2fm", target_alt)
    try:
        vehicle.simple_takeoff(target_alt)
        return True
    except Exception as exc:
        logger.error("simple_takeoff failed: %s", exc)
        return False


def wait_for_altitude(tof_bundle, target_alt: float, tolerance: float, timeout: float = 30.0) -> bool:
    logger.info("Waiting for ToF altitude %.2fm ± %.2fm", target_alt, tolerance)
    start = time.time()
    while time.time() - start < timeout:
        distance = read_tof_distance(tof_bundle)
        if distance is not None:
            logger.debug("Altitude reading %.3fm", distance)
            if abs(distance - target_alt) <= tolerance:
                logger.info("Altitude stable at %.3fm", distance)
                return True
        time.sleep(0.2)
    logger.warning("Altitude wait timed out")
    return False


def send_velocity_command(vehicle, vx: float, vy: float, vz: float, dry_run: bool) -> None:
    if dry_run:
        logger.debug("[dry-run] Velocity command vx=%.2f vy=%.2f vz=%.2f", vx, vy, vz)
        return
    yaw = math.radians(getattr(vehicle, "heading", 0.0))
    v_north = vx * math.cos(yaw) - vy * math.sin(yaw)
    v_east = vx * math.sin(yaw) + vy * math.cos(yaw)
    v_down = -vz
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        vehicle._master.target_system if hasattr(vehicle, "_master") else 1,
        vehicle._master.target_component if hasattr(vehicle, "_master") else 1,
        8,
        0b0000111111000111,
        0,
        0,
        0,
        v_north,
        v_east,
        v_down,
        0,
        0,
        0,
        0,
        0,
    )
    vehicle.send_mavlink(msg)


def hold_position(vehicle, duration: float, dry_run: bool) -> None:
    logger.info("Holding position for %ss", duration)
    end_time = time.time() + duration
    while time.time() < end_time:
        send_velocity_command(vehicle, 0.0, 0.0, 0.0, dry_run)
        time.sleep(1.0)


def guided_descent(vehicle, tof_bundle, ground_threshold: float, dry_run: bool) -> None:
    logger.info("Beginning slow descent")
    while True:
        distance = read_tof_distance(tof_bundle)
        if distance is None:
            send_velocity_command(vehicle, 0.0, 0.0, -0.2, dry_run)
            time.sleep(0.2)
            continue
        if distance <= ground_threshold * 2:
            logger.info("Reached %.2fm -> final landing phase", distance)
            break
        if distance > 0.6:
            vz = -0.35
        elif distance > 0.3:
            vz = -0.25
        else:
            vz = -0.12
        logger.debug("Descent ToF=%.3fm vz=%.2f", distance, vz)
        send_velocity_command(vehicle, 0.0, 0.0, vz, dry_run)
        time.sleep(0.2)


def final_touchdown(vehicle, tof_bundle, ground_threshold: float, dry_run: bool) -> None:
    logger.info("Final landing phase using throttle override")
    throttle = 1400
    while True:
        distance = read_tof_distance(tof_bundle)
        if distance is not None and distance <= ground_threshold:
            logger.info("Touchdown detected at %.3fm", distance)
            break
        throttle = max(1100, throttle - 5)
        apply_throttle_override(vehicle, throttle, dry_run)
        time.sleep(0.2)
    apply_throttle_override(vehicle, 1100, dry_run)
    time.sleep(0.5)
    clear_rc_override(vehicle, dry_run)


def disarm_vehicle(vehicle, dry_run: bool) -> None:
    if dry_run:
        logger.info("[dry-run] Would disarm vehicle")
        return
    logger.info("Disarming vehicle")
    vehicle.armed = False
    start = time.time()
    while vehicle.armed and time.time() - start < 5.0:
        time.sleep(0.2)


# ---------------------------------------------------------------------------
# Main flow
# ---------------------------------------------------------------------------

def run_sequence(args: argparse.Namespace) -> int:
    tof_bundle = None
    vehicle = None

    def handle_sigint(signum, frame):
        raise KeyboardInterrupt()

    signal.signal(signal.SIGINT, handle_sigint)

    try:
        tof_bundle = initialize_tof_sensor(args.tof_channel)
    except Exception as exc:
        logger.error("Failed to initialize ToF sensor: %s", exc)
        return 1

    try:
        if args.dry_run:
            logger.warning("Running in dry-run mode. No MAVLink commands will be sent.")
        else:
            logger.info("Connecting to %s @ %s baud", args.connect, args.baud)
            vehicle = connect(args.connect, baud=args.baud, wait_ready=True)
            logger.info("Connected to vehicle %s", vehicle.version)

        if not wait_until_armable(vehicle, timeout=30, dry_run=args.dry_run):
            return 1
        if not ensure_gps_lock(vehicle, min_fix=3, min_sat=6, dry_run=args.dry_run):
            return 1
        if not set_mode(vehicle, "GUIDED", args.dry_run):
            return 1
        if not arm_vehicle(vehicle, args.dry_run):
            return 1
        if not gradual_liftoff(vehicle, tof_bundle, args, args.dry_run):
            return 1
        if not command_takeoff(vehicle, args.target_alt, args.dry_run):
            return 1
        wait_for_altitude(tof_bundle, args.target_alt, ALTITUDE_TOLERANCE, timeout=30.0)
        hold_position(vehicle, args.hold_seconds, args.dry_run)
        guided_descent(vehicle, tof_bundle, args.ground_threshold, args.dry_run)
        final_touchdown(vehicle, tof_bundle, args.ground_threshold, args.dry_run)
        disarm_vehicle(vehicle, args.dry_run)
        logger.info("GUIDED liftoff test completed successfully")
        return 0
    except KeyboardInterrupt:
        logger.warning("Interrupted by user")
        return 1
    finally:
        if vehicle is not None:
            try:
                clear_rc_override(vehicle, False)
            except Exception:
                pass
            try:
                vehicle.close()
            except Exception:
                pass
        shutdown_tof(tof_bundle)


def main() -> int:
    args = parse_args()
    configure_logging(args.log_level)
    logger.info("Target altitude %.2fm | hold %ss | dry_run=%s", args.target_alt, args.hold_seconds, args.dry_run)
    return run_sequence(args)


if __name__ == "__main__":
    sys.exit(main())


