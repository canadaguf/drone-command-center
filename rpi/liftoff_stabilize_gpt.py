#!/usr/bin/env python3
"""
Indoor STABILIZE liftoff script powered by DroneKit + ToF distance sensor.

Sequence:
1. Initialize the VL53L1X distance sensor via the same multiplexer flow as tof_test_multiplexer.py
2. Connect to ArduCopter (serial defaults in rpi/drone_client/config/production.yaml)
3. Switch to STABILIZE, arm, ramp throttle slowly until ToF reports >10cm
4. Use a simple PID-like throttle controller (ToF only) to reach 1m, hold for 60s
5. Reduce throttle gradually to descend, detect touchdown, cut motors, disarm

Run with --dry-run to exercise the control loop without actually connecting to the vehicle.
"""
import collections
import collections.abc
if not hasattr(collections, 'MutableMapping'):
    collections.MutableMapping = collections.abc.MutableMapping

import argparse
import errno
import logging
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
# Defaults / constants
# ---------------------------------------------------------------------------

CONNECTION_STRING = "/dev/ttyAMA0"
BAUD_RATE = 256000
TARGET_ALTITUDE = 1.0
HOLD_DURATION = 60
GROUND_THRESHOLD = 0.08
TAKEOFF_THRESHOLD = 0.1
ALTITUDE_TOLERANCE = 0.05
THROTTLE_IDLE = 1100
THROTTLE_MAX = 1900
HOVER_THROTTLE = 1500
THROTTLE_STEP_UP = 6
THROTTLE_STEP_DOWN = 6
MAX_CLIMB_PWM = 1650
TOF_STALE_TIMEOUT = 0.6
MAX_HOVER_DELTA = 250

MULTIPLEXER_ADDRESS = 0x70
VL53L1X_ADDRESS = 0x29
BOTTOM_SENSOR_CHANNEL = 1
MAX_I2C_RETRIES = 5
BASE_RETRY_DELAY = 0.02
HEIGHT_LOG_INTERVAL = 1.0

KP_ASCEND = 15.0
KI_ASCEND = 2.0
KD_ASCEND = 5.0
KP_HOLD = 8.0
KI_HOLD = 1.0
KD_HOLD = 3.0

logger = logging.getLogger("liftoff_stabilize")
LAST_HEIGHT_LOG = 0.0


class AltitudeControlError(RuntimeError):
    """Raised when the altitude controller decides it is unsafe to continue."""

    pass



# ---------------------------------------------------------------------------
# CLI helpers
# ---------------------------------------------------------------------------

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Standalone STABILIZE liftoff script (ToF only altitude control)"
    )
    parser.add_argument("--connect", default=CONNECTION_STRING, help="MAVLink connection string")
    parser.add_argument("--baud", type=int, default=BAUD_RATE, help="Serial baud rate")
    parser.add_argument("--target-alt", type=float, default=TARGET_ALTITUDE, help="Target hover altitude (meters)")
    parser.add_argument("--hold-seconds", type=int, default=HOLD_DURATION, help="Hold duration at peak altitude")
    parser.add_argument("--ground-threshold", type=float, default=GROUND_THRESHOLD, help="Distance treated as touchdown (m)")
    parser.add_argument("--takeoff-threshold", type=float, default=TAKEOFF_THRESHOLD, help="Distance treated as liftoff (m)")
    parser.add_argument("--tof-channel", type=int, default=BOTTOM_SENSOR_CHANNEL, help="Multiplexer channel for bottom ToF")
    parser.add_argument("--idle-throttle", type=int, default=THROTTLE_IDLE, help="Initial throttle PWM for ramp sequence")
    parser.add_argument("--hover-throttle", type=int, default=HOVER_THROTTLE, help="Throttle guess for hovering")
    parser.add_argument("--max-throttle", type=int, default=THROTTLE_MAX, help="Upper PWM bound")
    parser.add_argument(
        "--max-climb-pwm",
        type=int,
        default=MAX_CLIMB_PWM,
        help="Upper PWM bound during the climb phase",
    )
    parser.add_argument("--kp", type=float, default=KP_ASCEND, help="PID proportional gain for climb/hold")
    parser.add_argument("--ki", type=float, default=KI_ASCEND, help="PID integral gain for climb/hold")
    parser.add_argument("--kd", type=float, default=KD_ASCEND, help="PID derivative gain for climb/hold")
    parser.add_argument("--throttle-step-up", type=int, default=THROTTLE_STEP_UP, help="PWM increment when below target altitude")
    parser.add_argument("--throttle-step-down", type=int, default=THROTTLE_STEP_DOWN, help="PWM decrement when above target altitude")
    parser.add_argument(
        "--tof-stale-timeout",
        type=float,
        default=TOF_STALE_TIMEOUT,
        help="Abort climb if no ToF readings arrive within this window (seconds)",
    )
    parser.add_argument(
        "--max-hover-delta",
        type=int,
        default=MAX_HOVER_DELTA,
        help="Abort climb if throttle exceeds hover guess by this delta without reaching altitude",
    )
    parser.add_argument("--dry-run", action="store_true", help="Skip DroneKit commands, only log the flow")
    parser.add_argument("--log-level", default="INFO", help="Logging level")
    return parser.parse_args()


def configure_logging(level: str) -> None:
    logging.basicConfig(
        level=getattr(logging, level.upper(), logging.INFO),
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )


# ---------------------------------------------------------------------------
# ToF helpers copied from rpi/tof_test_multiplexer.py
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
        log_height(distance_m)
        return distance_m
    except Exception as exc:
        logger.debug("ToF read error: %s", exc)
        return None


def log_height(distance: float) -> None:
    global LAST_HEIGHT_LOG
    now = time.time()
    if now - LAST_HEIGHT_LOG >= HEIGHT_LOG_INTERVAL:
        logger.info("Current height: %.3fm", distance)
        LAST_HEIGHT_LOG = now


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
# Drone helpers
# ---------------------------------------------------------------------------

def set_mode(vehicle, mode: str, dry_run: bool, timeout: float = 5.0) -> bool:
    if dry_run:
        logger.info("[dry-run] Would set mode %s", mode)
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


def ramp_until_takeoff(vehicle, tof_bundle, args, dry_run: bool) -> Optional[int]:
    logger.info("Ramping throttle until ToF exceeds %.2fm", args.takeoff_threshold)
    throttle = args.idle_throttle
    last_log = time.time()
    ceiling = min(args.max_climb_pwm, args.max_throttle)
    ceiling = max(ceiling, args.idle_throttle)
    while throttle <= ceiling:
        apply_throttle_override(vehicle, throttle, dry_run)
        time.sleep(0.2)
        distance = read_tof_distance(tof_bundle)
        if distance is not None and distance > args.takeoff_threshold:
            logger.info("Takeoff detected at %.3fm (throttle %s)", distance, throttle)
            return throttle
        throttle += 4
        if time.time() - last_log > 1.0:
            logger.debug("Throttle %s ToF=%s", throttle, f"{distance:.3f}" if distance else "None")
            last_log = time.time()
    logger.error("Climb PWM ceiling %s reached without liftoff", ceiling)
    return None


def altitude_pid_loop(
    vehicle,
    tof_bundle,
    target_alt: float,
    tolerance: float,
    gains: Dict[str, float],
    throttle_pwm: int,
    dry_run: bool,
    duration: Optional[float] = None,
    timeout: float = 30.0,
    hover_pwm: Optional[int] = None,
    max_hover_delta: Optional[int] = None,
    tof_stale_timeout: float = TOF_STALE_TIMEOUT,
) -> int:
    integral = 0.0
    last_error = 0.0
    dt = 0.2
    start = time.time()
    hold_start = None
    step_up = gains.get("step_up", THROTTLE_STEP_UP)
    step_down = gains.get("step_down", THROTTLE_STEP_DOWN)
    last_valid_sample = None

    while time.time() - start < timeout:
        distance = read_tof_distance(tof_bundle)
        now = time.time()
        if distance is None:
            if last_valid_sample is not None and now - last_valid_sample >= tof_stale_timeout:
                logger.error("ToF data unavailable for %.2fs; aborting altitude control", now - last_valid_sample)
                throttle_pwm = gains["min_pwm"]
                apply_throttle_override(vehicle, throttle_pwm, dry_run)
                raise AltitudeControlError("ToF data stale")
            throttle_pwm = max(gains["min_pwm"], throttle_pwm - step_down)
            apply_throttle_override(vehicle, throttle_pwm, dry_run)
            time.sleep(dt)
            continue

        last_valid_sample = now

        if hover_pwm is not None and max_hover_delta is not None:
            if throttle_pwm >= hover_pwm + max_hover_delta and distance < target_alt - tolerance:
                logger.error(
                    "Throttle %s exceeded hover guess (%s + %s) while altitude %.3fm < target %.3fm",
                    throttle_pwm,
                    hover_pwm,
                    max_hover_delta,
                    distance,
                    target_alt,
                )
                throttle_pwm = max(gains["min_pwm"], hover_pwm)
                apply_throttle_override(vehicle, throttle_pwm, dry_run)
                raise AltitudeControlError("Hover delta exceeded without climb")

        error = target_alt - distance
        integral += error * dt
        integral = max(-0.5, min(0.5, integral))
        derivative = (error - last_error) / dt
        last_error = error

        adjustment = (gains["kp"] * error + gains["ki"] * integral + gains["kd"] * derivative) * 0.1
        if distance < target_alt - tolerance:
            throttle_pwm = min(gains["max_pwm"], throttle_pwm + step_up)
        elif distance > target_alt + tolerance:
            throttle_pwm = max(gains["min_pwm"], throttle_pwm - step_down)
        else:
            throttle_pwm += int(adjustment)
        throttle_pwm = max(gains["min_pwm"], min(gains["max_pwm"], throttle_pwm))
        apply_throttle_override(vehicle, throttle_pwm, dry_run)

        if duration is None:
            if abs(error) <= tolerance:
                if hold_start is None:
                    hold_start = time.time()
                elif time.time() - hold_start > 0.8:
                    logger.info("Altitude %.3fm reached (target %.3fm)", distance, target_alt)
                    return throttle_pwm
            else:
                hold_start = None
        else:
            if hold_start is None:
                hold_start = time.time()
            elapsed = time.time() - hold_start
            if elapsed >= duration:
                logger.info("Hold complete (%.1fs)", elapsed)
                return throttle_pwm

        time.sleep(dt)

    logger.warning("PID loop timeout (target %.2fm)", target_alt)
    return throttle_pwm


def descend_to_ground(vehicle, tof_bundle, args, throttle_pwm: int, dry_run: bool) -> int:
    logger.info("Starting controlled descent")
    throttle = throttle_pwm
    decrement = 4
    while True:
        distance = read_tof_distance(tof_bundle)
        if distance is None:
            throttle = max(args.idle_throttle, throttle - decrement // 2)
            apply_throttle_override(vehicle, throttle, dry_run)
            time.sleep(0.2)
            continue

        if distance <= args.ground_threshold * 2:
            logger.info("Within %.3fm of ground, switching to soft landing", distance)
            return throttle

        if distance > 0.5:
            reduction = decrement * 2
        elif distance > 0.2:
            reduction = decrement
        elif distance > 0.12:
            reduction = decrement // 2
        else:
            reduction = decrement // 4

        throttle = max(args.idle_throttle, throttle - reduction)
        apply_throttle_override(vehicle, throttle, dry_run)
        time.sleep(0.2)


def land_and_cut(vehicle, tof_bundle, args, throttle_pwm: int, dry_run: bool) -> None:
    logger.info("Soft landing using ToF feedback")
    throttle = throttle_pwm
    decrement = 4
    while True:
        distance = read_tof_distance(tof_bundle)
        if distance is None:
            throttle = max(args.idle_throttle, throttle - decrement // 2)
        elif distance <= args.ground_threshold:
            logger.info("Touchdown detected at %.3fm", distance)
            throttle = args.idle_throttle
            break
        else:
            if distance > 0.2:
                reduction = decrement
            elif distance > 0.12:
                reduction = decrement // 2
            else:
                reduction = decrement // 4
            throttle = max(args.idle_throttle, throttle - reduction)
        apply_throttle_override(vehicle, throttle, dry_run)
        time.sleep(0.2)

    apply_throttle_override(vehicle, args.idle_throttle, dry_run)
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
    args.max_climb_pwm = min(args.max_climb_pwm, args.max_throttle)

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

        if not set_mode(vehicle, "STABILIZE", args.dry_run):
            return 1
        if not arm_vehicle(vehicle, args.dry_run):
            return 1

        throttle = ramp_until_takeoff(vehicle, tof_bundle, args, args.dry_run)
        if throttle is None:
            return 1

        gains_ascent = {
            "kp": args.kp,
            "ki": args.ki,
            "kd": args.kd,
            "min_pwm": args.idle_throttle,
            "max_pwm": args.max_climb_pwm,
            "step_up": args.throttle_step_up,
            "step_down": args.throttle_step_down,
        }
        gains_hold = {
            "kp": KP_HOLD,
            "ki": KI_HOLD,
            "kd": KD_HOLD,
            "min_pwm": args.idle_throttle,
            "max_pwm": args.max_throttle,
            "step_up": args.throttle_step_up,
            "step_down": args.throttle_step_down,
        }
        try:
            throttle = altitude_pid_loop(
                vehicle,
                tof_bundle,
                args.target_alt,
                ALTITUDE_TOLERANCE,
                gains_ascent,
                throttle,
                args.dry_run,
                hover_pwm=args.hover_throttle,
                max_hover_delta=args.max_hover_delta,
                tof_stale_timeout=args.tof_stale_timeout,
            )

            throttle = altitude_pid_loop(
                vehicle,
                tof_bundle,
                args.target_alt,
                ALTITUDE_TOLERANCE,
                gains_hold,
                throttle,
                args.dry_run,
                duration=args.hold_seconds,
                timeout=args.hold_seconds + 10,
                hover_pwm=args.hover_throttle,
                max_hover_delta=args.max_hover_delta,
                tof_stale_timeout=args.tof_stale_timeout,
            )
        except AltitudeControlError as exc:
            logger.error("Altitude controller aborted: %s", exc)
            apply_throttle_override(vehicle, args.idle_throttle, args.dry_run)
            time.sleep(0.5)
            clear_rc_override(vehicle, args.dry_run)
            disarm_vehicle(vehicle, args.dry_run)
            return 1

        throttle = descend_to_ground(vehicle, tof_bundle, args, throttle, args.dry_run)
        land_and_cut(vehicle, tof_bundle, args, throttle, args.dry_run)
        disarm_vehicle(vehicle, args.dry_run)
        logger.info("STABILIZE liftoff test completed successfully")
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


