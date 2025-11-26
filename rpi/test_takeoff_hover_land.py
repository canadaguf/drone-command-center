#!/usr/bin/env python3
"""
Standalone script for autonomous takeoff, hover, and landing sequence.

This script performs:
1. Takeoff to 1.5m using ToF downward sensor with incremental throttle
2. Hold position and altitude for 1 minute using velocity commands
3. Slow landing using incremental throttle with ToF sensor feedback

Usage:
    python test_takeoff_hover_land.py
"""

import sys
import time
import logging
import collections
from pathlib import Path

# Python 3.13 compatibility fix for dronekit
# collections.MutableMapping was removed in Python 3.13
if sys.version_info >= (3, 13):
    import collections.abc
    if not hasattr(collections, 'MutableMapping'):
        collections.MutableMapping = collections.abc.MutableMapping
    if not hasattr(collections, 'Mapping'):
        collections.Mapping = collections.abc.Mapping
    if not hasattr(collections, 'Callable'):
        collections.Callable = collections.abc.Callable

# Add parent directory to path to import drone_client modules
sys.path.insert(0, str(Path(__file__).parent))

from drone_client.config import Config
from drone_client.controllers.drone_controller import DroneController
from drone_client.sensors.tof_sensors import ToFSensorManager

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.StreamHandler(sys.stdout)
    ]
)
logger = logging.getLogger(__name__)


def main():
    """Main flight sequence."""
    config = None
    drone_controller = None
    tof_sensors = None
    
    try:
        # Step 1: Load configuration
        logger.info("=" * 60)
        logger.info("Starting autonomous flight sequence")
        logger.info("=" * 60)
        
        config_path = Path(__file__).parent / "drone_client" / "config" / "production.yaml"
        config = Config(config_path=str(config_path))
        logger.info(f"Configuration loaded from {config_path}")
        
        # Step 2: Initialize ToF sensors
        logger.info("\n[1/7] Initializing ToF sensors...")
        tof_config = config.get_tof_config()
        tof_sensors = ToFSensorManager(tof_config)
        
        if not tof_sensors.initialize():
            logger.error("Failed to initialize ToF sensors - aborting")
            return False
        
        logger.info("ToF sensors initialized successfully")
        
        # Step 3: Connect to drone
        logger.info("\n[2/7] Connecting to drone...")
        mavlink_config = config.get_mavlink_config()
        drone_controller = DroneController(mavlink_config)
        
        if not drone_controller.connect():
            logger.error("Failed to connect to drone - aborting")
            return False
        
        logger.info("Connected to drone successfully")
        logger.info(f"Vehicle mode: {drone_controller.get_mode()}")
        logger.info(f"Armed status: {drone_controller.is_armed()}")
        
        # Step 4: Arm the drone
        logger.info("\n[3/7] Arming drone...")
        if not drone_controller.is_armed():
            if not drone_controller.arm():
                logger.error("Failed to arm drone - aborting")
                return False
            logger.info("Drone armed successfully")
        else:
            logger.info("Drone already armed")
        
        # Step 5: Takeoff to 1.5m
        logger.info("\n[4/7] Taking off to 1.5m...")
        target_altitude = 1.5  # meters
        
        # Get bottom distance function for takeoff
        def get_bottom_distance():
            return tof_sensors.get_bottom_distance()
        
        success = drone_controller.incremental_throttle_takeoff(
            get_bottom_distance=get_bottom_distance,
            target_altitude=target_altitude,
            max_altitude=2.0,  # Safety limit
            ground_threshold=0.08,  # 8cm ground detection threshold
            throttle_increment=3,  # Small increments for smooth takeoff
            increment_interval=0.25,  # 0.25s between increments
            max_timeout=60.0  # 60 second timeout
        )
        
        if not success:
            logger.error("Takeoff failed - aborting")
            return False
        
        logger.info(f"Takeoff completed - reached {target_altitude}m")
        
        # Step 6: Hover for 1 minute with altitude monitoring
        logger.info("\n[5/7] Hovering for 60 seconds...")
        hover_duration = 60.0  # seconds
        hover_start_time = time.time()
        altitude_tolerance = 0.10  # 10cm tolerance for altitude monitoring
        last_altitude_log = 0.0
        
        while (time.time() - hover_start_time) < hover_duration:
            elapsed = time.time() - hover_start_time
            remaining = hover_duration - elapsed
            
            # Read current altitude
            current_altitude = tof_sensors.get_bottom_distance()
            
            if current_altitude is not None:
                # Log altitude every 5 seconds or if significant change
                if (elapsed - last_altitude_log) >= 5.0 or abs(current_altitude - target_altitude) > altitude_tolerance:
                    logger.info(f"Hover: {elapsed:.1f}s elapsed, {remaining:.1f}s remaining | "
                              f"Altitude: {current_altitude:.3f}m (target: {target_altitude}m)")
                    last_altitude_log = elapsed
                
                # If altitude deviates significantly, send small correction
                altitude_error = current_altitude - target_altitude
                if abs(altitude_error) > altitude_tolerance:
                    # Small vertical velocity correction
                    correction_vz = -0.1 * altitude_error  # Negative because vz is down in NED frame
                    correction_vz = max(-0.3, min(0.3, correction_vz))  # Limit to ±0.3 m/s
                    drone_controller.send_velocity_command(0.0, 0.0, correction_vz, 0.0)
                else:
                    # Maintain position with zero velocity
                    drone_controller.send_velocity_command(0.0, 0.0, 0.0, 0.0)
            else:
                logger.warning("ToF sensor reading failed during hover - maintaining zero velocity")
                # Continue with zero velocity if sensor fails
                drone_controller.send_velocity_command(0.0, 0.0, 0.0, 0.0)
            
            time.sleep(0.2)  # Update every 200ms
        
        logger.info("Hover completed - 60 seconds elapsed")
        
        # Step 7: Land using incremental throttle
        logger.info("\n[6/7] Landing...")
        success = drone_controller.incremental_throttle_land(
            get_bottom_distance=get_bottom_distance,
            ground_threshold=0.08,  # 8cm ground detection threshold
            throttle_decrement=4,  # Small decrements for smooth landing
            decrement_interval=0.15,  # 0.15s between decrements
            min_throttle=1100,  # Minimum throttle
            max_timeout=60.0  # 60 second timeout
        )
        
        if not success:
            logger.error("Landing failed - attempting emergency disarm")
            drone_controller.disarm()
            return False
        
        logger.info("Landing completed successfully")
        
        # Step 8: Disarm and cleanup
        logger.info("\n[7/7] Disarming and cleaning up...")
        if drone_controller.is_armed():
            if not drone_controller.disarm():
                logger.warning("Failed to disarm - vehicle may still be armed")
            else:
                logger.info("Drone disarmed successfully")
        
        logger.info("=" * 60)
        logger.info("Flight sequence completed successfully!")
        logger.info("=" * 60)
        return True
        
    except KeyboardInterrupt:
        logger.warning("\nFlight sequence interrupted by user")
        if drone_controller and drone_controller.is_connected():
            if drone_controller.is_armed():
                logger.info("Emergency landing and disarm...")
                try:
                    # Try to land first
                    if tof_sensors:
                        def get_bottom_distance():
                            return tof_sensors.get_bottom_distance()
                        drone_controller.incremental_throttle_land(
                            get_bottom_distance=get_bottom_distance,
                            max_timeout=30.0
                        )
                except:
                    pass
                finally:
                    drone_controller.disarm()
        return False
        
    except Exception as e:
        logger.error(f"Unexpected error during flight sequence: {e}")
        import traceback
        logger.error(traceback.format_exc())
        
        # Emergency cleanup
        if drone_controller and drone_controller.is_connected():
            try:
                if drone_controller.is_armed():
                    logger.info("Emergency disarm...")
                    drone_controller.disarm()
            except:
                pass
        
        return False
        
    finally:
        # Cleanup
        logger.info("Cleaning up resources...")
        if tof_sensors:
            try:
                tof_sensors.cleanup()
            except Exception as e:
                logger.warning(f"Error cleaning up ToF sensors: {e}")
        
        if drone_controller:
            try:
                drone_controller.disconnect()
            except Exception as e:
                logger.warning(f"Error disconnecting from drone: {e}")


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)

