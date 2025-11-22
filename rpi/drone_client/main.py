"""
Main orchestrator for autonomous drone client.
Coordinates all subsystems and manages async event loops.
"""

# Python 3.13 compatibility fix for DroneKit
# collections.MutableMapping was removed in Python 3.13
import collections
import collections.abc
if not hasattr(collections, 'MutableMapping'):
    collections.MutableMapping = collections.abc.MutableMapping

import asyncio
import logging
import signal
import sys
import time
from typing import Dict, Any, Optional

# Import core components
from .config import Config
from .controllers.drone_controller import DroneController
from .controllers.telemetry_reader import TelemetryReader
from .sensors.telemetry import TelemetryCollector
from .communication.websocket_client import WebSocketClient

# Vision imports (stubbed - disabled for now)
# from .vision.yolo_detector import YOLODetector
# from .vision.person_tracker import PersonTracker
# from .vision.depth_estimator import DepthEstimator
# from .sensors.camera import CameraManager

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(name)s: %(message)s',
    handlers=[
        logging.StreamHandler(),
        logging.FileHandler('/var/log/drone_client.log')
    ]
)
logger = logging.getLogger(__name__)


class DroneClient:
    """Main drone client orchestrator."""
    
    def __init__(self, config_path: Optional[str] = None):
        """Initialize drone client.
        
        Args:
            config_path: Path to configuration file
        """
        # Load configuration
        self.config = Config(config_path)
        
        # Initialize core components
        self.drone_controller = None
        self.telemetry_reader = None
        self.telemetry = None
        self.websocket = None
        
        # Vision components (stubbed - disabled)
        # self.camera = None
        # self.yolo_detector = None
        # self.person_tracker = None
        # self.depth_estimator = None
        
        # State
        self.running = False
        self.initialized = False
        
        # Event loops
        self.telemetry_loop_task = None
        self.command_loop_task = None
        
        # Setup signal handlers
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
    
    def _signal_handler(self, signum, frame):
        """Handle shutdown signals."""
        logger.info(f"Received signal {signum} (Ctrl+C), shutting down gracefully...")
        self.running = False
    
    async def initialize(self) -> bool:
        """Initialize all components.
        
        Returns:
            True if initialization successful, False otherwise
        """
        try:
            logger.info("Initializing drone client...")
            
            # Initialize DroneKit controller
            mavlink_config = self.config.get_mavlink_config()
            self.drone_controller = DroneController(mavlink_config)
            if not self.drone_controller.connect():
                logger.error("Failed to connect to flight controller")
                return False
            
            # Initialize pymavlink telemetry reader
            self.telemetry_reader = TelemetryReader(mavlink_config)
            if not self.telemetry_reader.connect():
                logger.error("Failed to connect to MAVLink for telemetry")
                # Don't fail completely - telemetry is important but not critical for basic control
                logger.warning("Continuing without telemetry reader - some features may be limited")
            
            # Initialize telemetry collector
            self.telemetry = TelemetryCollector()
            
            # Initialize WebSocket client
            backend_config = self.config.get_backend_config()
            self.websocket = WebSocketClient(backend_config)
            
            # Vision components (stubbed - disabled for now)
            # logger.info("Vision features disabled - skipping initialization")
            # self.camera = None
            # self.yolo_detector = None
            # self.person_tracker = None
            # self.depth_estimator = None
            
            # Setup callbacks
            self._setup_callbacks()
            
            self.initialized = True
            logger.info("Drone client initialized successfully")
            return True
            
        except Exception as e:
            logger.error(f"Failed to initialize drone client: {e}")
            import traceback
            logger.error(traceback.format_exc())
            return False
    
    def _setup_callbacks(self) -> None:
        """Setup component callbacks."""
        # WebSocket callbacks
        self.websocket.set_callbacks(
            on_connect=self._on_websocket_connect,
            on_disconnect=self._on_websocket_disconnect,
            on_command=self._on_command_received
        )
        
        # Set command executor to execute commands (async function)
        self.websocket.set_command_executor(self._on_command_received)
    
    async def start(self) -> None:
        """Start drone client."""
        if not self.initialized:
            logger.error("Drone client not initialized")
            return
        
        self.running = True
        logger.info("Starting drone client...")
        
        # Start WebSocket connection
        await self.websocket.connect()
        
        # Start async tasks
        tasks = [
            self._telemetry_loop(),
            self._command_loop(),
            self.websocket.start()
        ]
        
        try:
            # Wait for all tasks
            await asyncio.gather(*tasks, return_exceptions=True)
        except KeyboardInterrupt:
            logger.info("KeyboardInterrupt caught in start()")
            self.running = False
        except Exception as e:
            logger.error(f"Error in main loop: {e}")
            import traceback
            logger.error(traceback.format_exc())
        finally:
            await self.shutdown()
    
    async def _telemetry_loop(self) -> None:
        """Telemetry streaming loop."""
        logger.info("Telemetry loop started")
        
        while self.running:
            try:
                # Read telemetry from pymavlink
                if self.telemetry_reader and self.telemetry_reader.is_connected():
                    mavlink_data = self.telemetry_reader.get_telemetry()
                    self.telemetry.update_from_mavlink(mavlink_data)
                else:
                    # Fallback to DroneKit telemetry if pymavlink not available
                    if self.drone_controller and self.drone_controller.is_connected():
                        dronekit_data = self.drone_controller.get_telemetry()
                        # Convert DroneKit format to MAVLink format
                        mavlink_data = self._convert_dronekit_to_mavlink(dronekit_data)
                        self.telemetry.update_from_mavlink(mavlink_data)
                
                # Update tracking status (stubbed)
                self.telemetry.update_tracking_status("DISCONNECTED")
                
                # Get telemetry data
                telemetry_data = self.telemetry.get_telemetry()
                
                # Send to backend (only if websocket is connected)
                if self.websocket and self.websocket.connected:
                    try:
                        await self.websocket.send_telemetry(telemetry_data)
                    except Exception as e:
                        logger.error(f"Error sending telemetry: {e}")
                else:
                    logger.debug("WebSocket not connected, skipping telemetry send")
                
                await asyncio.sleep(0.5)  # 2 Hz
                
            except Exception as e:
                logger.error(f"Error in telemetry loop: {e}")
                await asyncio.sleep(0.5)
    
    def _convert_dronekit_to_mavlink(self, dronekit_data: Dict[str, Any]) -> Dict[str, Any]:
        """Convert DroneKit telemetry format to MAVLink format.
        
        Args:
            dronekit_data: Telemetry data from DroneKit
            
        Returns:
            Dictionary in MAVLink format
        """
        mavlink_data = {}
        
        if 'location' in dronekit_data:
            loc = dronekit_data['location']
            mavlink_data['lat'] = loc.get('lat')
            mavlink_data['lon'] = loc.get('lon')
            mavlink_data['altitude'] = loc.get('alt')
        
        if 'relative_alt' in dronekit_data:
            mavlink_data['relative_alt'] = dronekit_data['relative_alt']
        
        if 'battery' in dronekit_data:
            bat = dronekit_data['battery']
            mavlink_data['battery_voltage'] = bat.get('voltage')
            mavlink_data['battery_remaining'] = bat.get('level')
            mavlink_data['battery_current'] = bat.get('current')
        
        if 'mode' in dronekit_data:
            mavlink_data['mode'] = dronekit_data['mode']
        
        if 'armed' in dronekit_data:
            mavlink_data['armed'] = dronekit_data['armed']
        
        if 'groundspeed' in dronekit_data:
            mavlink_data['groundspeed'] = dronekit_data['groundspeed']
        
        if 'heading' in dronekit_data:
            mavlink_data['heading'] = dronekit_data['heading']
        
        return mavlink_data
    
    async def _command_loop(self) -> None:
        """Command processing loop."""
        logger.info("Command loop started")
        
        while self.running:
            try:
                # Process any pending commands
                # Commands are handled by WebSocket client callbacks
                await asyncio.sleep(0.1)
                
            except Exception as e:
                logger.error(f"Error in command loop: {e}")
                await asyncio.sleep(0.1)
    
    async def shutdown(self) -> None:
        """Shutdown drone client."""
        logger.info("Shutting down drone client...")
        
        self.running = False
        
        # Disconnect WebSocket
        if self.websocket:
            await self.websocket.disconnect()
        
        # Disconnect telemetry reader
        if self.telemetry_reader:
            self.telemetry_reader.disconnect()
        
        # Disconnect drone controller
        if self.drone_controller:
            self.drone_controller.disconnect()
        
        # Cleanup camera (if vision was enabled)
        # if self.camera:
        #     self.camera.cleanup()
        
        logger.info("Drone client shutdown complete")
    
    # Callback methods
    def _on_websocket_connect(self) -> None:
        """Handle WebSocket connection."""
        logger.info("WebSocket connected")
    
    def _on_websocket_disconnect(self) -> None:
        """Handle WebSocket disconnection."""
        logger.warning("WebSocket disconnected")
    
    async def _on_command_received(self, command: str, payload: Dict[str, Any]) -> Dict[str, Any]:
        """Handle command received.
        
        Args:
            command: Command name
            payload: Command payload
            
        Returns:
            Dictionary with 'success' (bool) and 'message' (str) keys
        """
        logger.info(f"Command received: {command}")
        
        try:
            if command == 'arm':
                return await self._handle_arm()
                    
            elif command == 'disarm':
                return await self._handle_disarm()
                
            elif command == 'takeoff':
                altitude = payload.get('altitude') if payload else None
                return await self._handle_takeoff(altitude)
                    
            elif command == 'land':
                return await self._handle_land()
                    
            elif command == 'freeze':
                return await self._handle_freeze()
                
            elif command == 'check_connection':
                return await self._handle_check_connection()
                
            elif command == 'prearm_checks':
                return await self._handle_prearm_checks()
                
            elif command == 'follow':
                # Stubbed - vision features disabled
                return {'success': False, 'message': 'Follow command not available - vision features disabled'}
                
            elif command == 'stop_following':
                # Stubbed - vision features disabled
                return {'success': False, 'message': 'Stop following command not available - vision features disabled'}
                
            elif command == 'set_distance_mode':
                # Stubbed - vision features disabled
                return {'success': False, 'message': 'Distance mode command not available - vision features disabled'}
            else:
                return {'success': False, 'message': f'Unknown command: {command}'}
                
        except Exception as e:
            logger.error(f"Error executing command {command}: {e}")
            import traceback
            logger.debug(traceback.format_exc())
            return {'success': False, 'message': f'Error executing command: {str(e)}'}
    
    async def _handle_arm(self) -> Dict[str, Any]:
        """Handle arm command."""
        if not self.drone_controller or not self.drone_controller.is_connected():
            return {'success': False, 'message': 'Drone controller not connected'}
        
        # Check if already armed
        if self.drone_controller.is_armed():
            return {'success': True, 'message': 'Drone already armed', 'armed': True}
        
        # Arm the vehicle
        success = self.drone_controller.arm()
        if not success:
            return {'success': False, 'message': 'Arm command failed - check connection and pre-arm conditions'}
        
        # Capture altitude reference when arming
        if self.telemetry_reader and self.telemetry_reader.is_connected():
            mavlink_data = self.telemetry_reader.get_telemetry()
            self.telemetry.update_from_mavlink(mavlink_data)
        elif self.drone_controller:
            dronekit_data = self.drone_controller.get_telemetry()
            mavlink_data = self._convert_dronekit_to_mavlink(dronekit_data)
            self.telemetry.update_from_mavlink(mavlink_data)
        
        if not self.telemetry.altitude_reference_captured:
            self.telemetry.capture_altitude_reference()
            logger.info("Altitude reference captured at arming")
        
        return {'success': True, 'message': 'Drone armed successfully', 'armed': True}
    
    async def _handle_disarm(self) -> Dict[str, Any]:
        """Handle disarm command."""
        if not self.drone_controller or not self.drone_controller.is_connected():
            return {'success': False, 'message': 'Drone controller not connected'}
        
        # Check if already disarmed
        if not self.drone_controller.is_armed():
            return {'success': True, 'message': 'Drone already disarmed', 'armed': False}
        
        # Disarm the vehicle
        success = self.drone_controller.disarm()
        if not success:
            return {'success': False, 'message': 'Disarm command failed - check connection'}
        
        # Reset altitude reference on disarm
        self.telemetry.reset_altitude_reference()
        logger.info("Altitude reference reset after disarm")
        
        return {'success': True, 'message': 'Drone disarmed successfully', 'armed': False}
    
    async def _handle_takeoff(self, altitude: Optional[float] = None) -> Dict[str, Any]:
        """Handle takeoff command."""
        if not self.drone_controller or not self.drone_controller.is_connected():
            return {'success': False, 'message': 'Drone controller not connected'}
        
        # Check if armed
        if not self.drone_controller.is_armed():
            return {'success': False, 'message': 'Cannot takeoff - drone not armed'}
        
        # Ensure altitude reference is captured before takeoff
        if not self.telemetry.altitude_reference_captured:
            if self.telemetry_reader and self.telemetry_reader.is_connected():
                mavlink_data = self.telemetry_reader.get_telemetry()
                self.telemetry.update_from_mavlink(mavlink_data)
            elif self.drone_controller:
                dronekit_data = self.drone_controller.get_telemetry()
                mavlink_data = self._convert_dronekit_to_mavlink(dronekit_data)
                self.telemetry.update_from_mavlink(mavlink_data)
            
            if not self.telemetry.capture_altitude_reference():
                logger.warning("Failed to capture altitude reference - takeoff may be inaccurate")
        
        # Get takeoff altitude from config or parameter
        takeoff_altitude = altitude
        if takeoff_altitude is None:
            takeoff_altitude = self.config.get_mavlink_config().get('takeoff_altitude', 1.5)
        
        # Use DroneKit simple_takeoff
        logger.info(f"Starting takeoff to {takeoff_altitude}m using DroneKit")
        success = self.drone_controller.simple_takeoff(takeoff_altitude)
        
        if success:
            return {'success': True, 'message': f'Takeoff to {takeoff_altitude}m started'}
        else:
            return {'success': False, 'message': 'Takeoff command failed'}
    
    async def _handle_land(self) -> Dict[str, Any]:
        """Handle land command."""
        if not self.drone_controller or not self.drone_controller.is_connected():
            return {'success': False, 'message': 'Drone controller not connected'}
        
        # Use DroneKit simple_land
        logger.info("Starting landing sequence using DroneKit")
        success = self.drone_controller.simple_land()
        
        if success:
            return {'success': True, 'message': 'Landing sequence started'}
        else:
            return {'success': False, 'message': 'Landing command failed'}
    
    async def _handle_freeze(self) -> Dict[str, Any]:
        """Handle freeze/loiter command."""
        if not self.drone_controller or not self.drone_controller.is_connected():
            return {'success': False, 'message': 'Drone controller not connected'}
        
        # Set LOITER mode using DroneKit
        success = self.drone_controller.set_mode_loiter()
        if success:
            # Clear any RC overrides
            self.drone_controller.clear_rc_override()
            return {'success': True, 'message': 'Loiter mode activated'}
        else:
            return {'success': False, 'message': 'Loiter command failed'}
    
    async def _handle_check_connection(self) -> Dict[str, Any]:
        """Handle check connection command."""
        drone_connected = self.drone_controller and self.drone_controller.is_connected()
        telemetry_connected = self.telemetry_reader and self.telemetry_reader.is_connected()
        websocket_connected = self.websocket and self.websocket.connected
        
        status = {
            'drone_controller': drone_connected,
            'telemetry_reader': telemetry_connected,
            'websocket': websocket_connected
        }
        
        all_connected = all(status.values())
        
        return {
            'success': all_connected,
            'message': f'Connection status: {status}',
            'status': status
        }
    
    async def _handle_prearm_checks(self) -> Dict[str, Any]:
        """Handle prearm checks command."""
        if not self.drone_controller or not self.drone_controller.is_connected():
            return {'success': False, 'message': 'Drone controller not connected'}
        
        checks = {}
        
        # Check GPS lock
        if self.telemetry_reader and self.telemetry_reader.is_connected():
            mavlink_data = self.telemetry_reader.get_telemetry()
            gps_fix = mavlink_data.get('gps_fix_type', 0)
            gps_satellites = mavlink_data.get('gps_satellites', 0)
            checks['gps'] = {
                'fix_type': gps_fix,
                'satellites': gps_satellites,
                'ok': gps_fix >= 2 and gps_satellites >= 6
            }
        else:
            checks['gps'] = {'ok': None, 'message': 'Telemetry reader not connected'}
        
        # Check battery
        if self.telemetry_reader and self.telemetry_reader.is_connected():
            mavlink_data = self.telemetry_reader.get_telemetry()
            battery_voltage = mavlink_data.get('battery_voltage')
            battery_remaining = mavlink_data.get('battery_remaining')
            checks['battery'] = {
                'voltage': battery_voltage,
                'remaining': battery_remaining,
                'ok': battery_voltage is not None and (battery_remaining is None or battery_remaining > 20)
            }
        else:
            checks['battery'] = {'ok': None, 'message': 'Telemetry reader not connected'}
        
        # Check armed status
        checks['armed'] = {
            'current': self.drone_controller.is_armed(),
            'ok': not self.drone_controller.is_armed()  # Should be disarmed for prearm checks
        }
        
        # Check mode
        mode = self.drone_controller.get_mode()
        checks['mode'] = {
            'current': mode,
            'ok': mode is not None
        }
        
        all_ok = all(check.get('ok', False) for check in checks.values() if check.get('ok') is not False)
        
        return {
            'success': all_ok,
            'message': 'Prearm checks completed',
            'checks': checks
        }


async def main():
    """Main entry point."""
    drone_client = None
    try:
        # Create and initialize drone client
        drone_client = DroneClient()
        
        if not await drone_client.initialize():
            logger.error("Failed to initialize drone client")
            sys.exit(1)
        
        # Start drone client
        await drone_client.start()
        
    except KeyboardInterrupt:
        logger.info("Interrupted by user (Ctrl+C)")
        if drone_client:
            await drone_client.shutdown()
    except Exception as e:
        logger.error(f"Fatal error: {e}")
        import traceback
        logger.error(traceback.format_exc())
        if drone_client:
            await drone_client.shutdown()
        sys.exit(1)


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logger.info("Force exit (Ctrl+C pressed twice)")
        sys.exit(0)
