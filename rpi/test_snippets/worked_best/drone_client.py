#!/usr/bin/env python3
# Python 3.13 compatibility fix for DroneKit
import collections
import collections.abc
if not hasattr(collections, 'MutableMapping'):
    collections.MutableMapping = collections.abc.MutableMapping

import asyncio
import websockets
import json
import logging
import signal
import sys
from dronekit import connect, VehicleMode

# --- Configuration ---
RENDER_WS_URL = "wss://drone-command-center.onrender.com/ws?client=drone"
MAVLINK_CONNECTION_STRING = "/dev/ttyAMA0"  # or "/dev/serial0"
MAVLINK_BAUD = 256000
TAKEOFF_ALTITUDE = 1.5  # meters

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s"
)
logger = logging.getLogger("drone-client")

# Global state
vehicle = None
websocket_conn = None
running = True

# Graceful shutdown
def signal_handler(sig, frame):
    global running, vehicle
    logger.info("Shutting down gracefully...")
    running = False
    if vehicle:
        try:
            vehicle.close()
        except:
            pass
    if websocket_conn:
        asyncio.create_task(websocket_conn.close())
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

# Initialize DroneKit connection
def init_dronekit():
    global vehicle
    logger.info(f"Connecting to flight controller at {MAVLINK_CONNECTION_STRING}...")
    # For serial connections, pass port as first arg and baud as keyword arg
    vehicle = connect(
        MAVLINK_CONNECTION_STRING,
        baud=MAVLINK_BAUD,
        wait_ready=False,
        timeout=10
    )
    vehicle.wait_ready(['autopilot_version'], timeout=10)
    logger.info(f"✓ Connected! Vehicle type: {vehicle.vehicle_type}")
    return vehicle

# Handle a single command from the web
async def handle_web_command(payload: dict, websocket):
    global vehicle
    action = payload.get("action")
    logger.info(f"Executing command: {action}")

    try:
        if action == "arm":
            if not vehicle.armed:
                vehicle.armed = True
                # Wait for arming
                while not vehicle.armed:
                    await asyncio.sleep(0.1)
            response = {"source": "drone", "type": "arm_success", "message": "Drone armed"}
            
        elif action == "disarm":
            if vehicle.armed:
                vehicle.armed = False
                # Wait for disarming
                while vehicle.armed:
                    await asyncio.sleep(0.1)
            response = {"source": "drone", "type": "disarm_success", "message": "Drone disarmed"}
            
        elif action == "takeoff":
            if not vehicle.armed:
                response = {"source": "drone", "type": "error", "message": "Drone must be armed before takeoff"}
            else:
                # Switch to GUIDED mode if not already
                if vehicle.mode.name != 'GUIDED':
                    vehicle.mode = VehicleMode('GUIDED')
                    await asyncio.sleep(1)
                # Takeoff
                vehicle.simple_takeoff(TAKEOFF_ALTITUDE)
                response = {"source": "drone", "type": "takeoff_success", "message": f"Takeoff to {TAKEOFF_ALTITUDE}m initiated"}
                
        elif action == "land":
            vehicle.mode = VehicleMode('LAND')
            response = {"source": "drone", "type": "land_success", "message": "Landing initiated"}
            
        elif action == "freeze":
            vehicle.mode = VehicleMode('LOITER')
            response = {"source": "drone", "type": "freeze_success", "message": "Loiter mode activated"}
            
        elif action == "check_connection":
            if vehicle and vehicle.armed is not None:
                response = {"source": "drone", "type": "connection_success", "message": "Connected to FC"}
            else:
                response = {"source": "drone", "type": "error", "message": "Not connected to FC"}
                
        elif action == "prearm_checks":
            # Basic pre-arm checks
            checks_passed = True
            issues = []
            
            if vehicle.gps_0 and vehicle.gps_0.fix_type < 2:
                checks_passed = False
                issues.append("GPS fix required")
            
            if vehicle.battery and vehicle.battery.voltage < 10.0:
                checks_passed = False
                issues.append("Low battery voltage")
            
            if checks_passed:
                response = {"source": "drone", "type": "prearm_success", "message": "Pre-arm checks passed"}
            else:
                response = {"source": "drone", "type": "prearm_failed", "message": f"Pre-arm checks failed: {', '.join(issues)}"}
        else:
            response = {"source": "drone", "type": "error", "message": f"Unknown action: {action}"}

    except Exception as e:
        logger.error(f"Command execution failed: {e}")
        response = {"source": "drone", "type": "error", "message": str(e)}

    # Send acknowledgment back to web clients
    await websocket.send(json.dumps(response))

# Main WebSocket loop
async def drone_websocket_client():
    global websocket_conn, running
    while running:
        try:
            logger.info(f"Connecting to Render backend: {RENDER_WS_URL}")
            async with websockets.connect(RENDER_WS_URL) as ws:
                websocket_conn = ws
                logger.info("? Connected to Render backend")

                # Reinitialize DroneKit on each reconnect (optional but safe)
                init_dronekit()

                while running:
                    try:
                        message = await ws.recv()
                        logger.debug(f"< Received: {message}")
                        data = json.loads(message)

                        # Only process commands from web
                        if data.get("source") == "web" and data.get("type") == "command":
                            await handle_web_command(data.get("payload", {}), ws)

                    except websockets.exceptions.ConnectionClosed:
                        logger.warning("WebSocket closed by server")
                        break
                    except json.JSONDecodeError:
                        logger.error("Invalid JSON received")
                    except Exception as e:
                        logger.error(f"Error in message loop: {e}")

        except Exception as e:
            logger.error(f"WebSocket connection failed: {e}")
            logger.info("Reconnecting in 5 seconds...")
            await asyncio.sleep(5)

# Entry point
if __name__ == "__main__":
    logger.info("Starting drone client...")
    try:
        asyncio.run(drone_websocket_client())
    except KeyboardInterrupt:
        logger.info("Interrupted by user")
    finally:
        logger.info("Drone client stopped.")
        
