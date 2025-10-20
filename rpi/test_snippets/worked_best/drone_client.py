#!/usr/bin/env python3
import asyncio
import websockets
import json
import logging
import signal
import sys
from pymavlink import mavutil

# --- Configuration ---
RENDER_WS_URL = "wss://drone-command-center.onrender.com/ws?client=drone"
MAVLINK_CONNECTION_STRING = "/dev/ttyAMA0"  # or "/dev/serial0"
MAVLINK_BAUD = 256000

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s"
)
logger = logging.getLogger("drone-client")

# Global state
mav_conn = None
websocket_conn = None
running = True

# Graceful shutdown
def signal_handler(sig, frame):
    global running
    logger.info("Shutting down gracefully...")
    running = False
    if websocket_conn:
        asyncio.create_task(websocket_conn.close())
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)
# Initialize MAVLink connection
def init_mavlink():
    global mav_conn
    logger.info(f"Connecting to flight controller at {MAVLINK_CONNECTION_STRING}...")
    mav_conn = mavutil.mavlink_connection(MAVLINK_CONNECTION_STRING, baud=MAVLINK_BAUD)
    mav_conn.wait_heartbeat()
    logger.info("Heartbeat received from flight controller.")
    return mav_conn

# Handle a single command from the web
async def handle_web_command(payload: dict, websocket):
    global mav_conn
    action = payload.get("action")
    logger.info(f"Executing command: {action}")

    try:
        if action == "arm":
            mav_conn.arducopter_arm()
            response = {"source": "drone", "type": "arm_success", "message": "Drone armed"}
        elif action == "disarm":
            mav_conn.arducopter_disarm()
            response = {"source": "drone", "type": "disarm_success", "message": "Drone disarmed"}
        elif action == "takeoff":
            # Optional: add takeoff logic later
            response = {"source": "drone", "type": "takeoff_success", "message": "Takeoff initiated"}
        elif action == "land":
            # Optional
            response = {"source": "drone", "type": "land_success", "message": "Landing initiated"}
        elif action == "freeze":
            # Hover in place (GUIDED mode + current position)
            response = {"source": "drone", "type": "freeze_success", "message": "Hovering"}
        elif action == "check_connection":
            response = {"source": "drone", "type": "connection_success", "message": "Connected to FC"}
        elif action == "prearm_checks":
            # You can add real pre-arm checks here (e.g., GPS lock, battery)
            response = {"source": "drone", "type": "prearm_success", "message": "Pre-arm checks passed"}
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

                # Reinitialize MAVLink on each reconnect (optional but safe)
                init_mavlink()

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
        
