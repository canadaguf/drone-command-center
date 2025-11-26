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

# Vision imports
from .vision.yolo_detector import YOLODetector
from .vision.object_tracker import ObjectTracker
from .controllers.tracking_controller import TrackingController
from .sensors.camera import CameraManager
from .sensors.tof_sensors import ToFSensorManager

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
        
        # Vision components
        self.camera = None
        self.yolo_detector = None
        self.object_tracker = None
        self.tracking_controller = None
        
        # ToF sensors
        self.tof_sensors = None
        
        # State
        self.running = False
        self.initialized = False
        
        # Event loops
        self.telemetry_loop_task = None
        self.command_loop_task = None
        self.vision_loop_task = None
        self.tof_loop_task = None
        
        # Setup signal handlers
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
    
    def _signal_handler(self, signum, frame):
        """Handle shutdown signals."""
        logger.info(f"Received signal {signum} (Ctrl+C), shutting down gracefully...")
        self.running = False
    
    def _enforce_height_limit(self, vz: float, max_height: float = 2.0) -> float:
        """Enforce maximum height limit using bottom ToF sensor.
        
        Args:
            vz: Desired vertical velocity (m/s, positive = up)
            max_height: Maximum allowed height in meters (default 2.0m)
            
        Returns:
            Adjusted vertical velocity (may be reduced or set to negative)
        """
        if not self.tof_sensors:
            return vz  # No sensor, can't enforce
        
        current_height = self.tof_sensors.get_bottom_distance()
        
        if current_height is None:
            # Sensor unavailable - be conservative
            if vz > 0:
                logger.warning("Height limit check: sensor unavailable, reducing upward velocity")
                return vz * 0.5  # Reduce upward velocity
            return vz
        
        # Check if at or above max height
        if current_height >= max_height:
            if vz > 0:
                logger.warning(f"Height limit exceeded ({current_height:.3f}m >= {max_height}m) - preventing ascent")
                return 0.0  # Stop upward movement
            # Allow downward movement
            return vz
        
        # Check if approaching limit (within 0.2m)
        if current_height >= (max_height - 0.2):
            if vz > 0:
                logger.debug(f"Approaching height limit ({current_height:.3f}m) - reducing upward velocity")
                return vz * 0.3  # Reduce upward velocity
        
        return vz
    
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
            
            # Initialize ToF sensors
            tof_config = self.config.get_tof_config()
            self.tof_sensors = ToFSensorManager(tof_config)
            if not self.tof_sensors.initialize():
                logger.warning("Failed to initialize ToF sensors - continuing without them")
                logger.warning("Height control and distance tracking may be limited")
                self.tof_sensors = None
            
            # Initialize WebSocket client
            backend_config = self.config.get_backend_config()
            self.websocket = WebSocketClient(backend_config)
            
            # Initialize vision components
            camera_config = self.config.get_camera_config()
            self.camera = CameraManager(camera_config)
            if not self.camera.initialize():
                logger.error("Failed to initialize camera")
                return False
            
            # Initialize YOLO detector
            vision_config = self.config.get_vision_config()
            model_path = vision_config.get('model_path', '/home/pi/models/yolo11n.onnx')
            try:
                self.yolo_detector = YOLODetector(
                    model_path=model_path,
                    input_size=vision_config.get('input_size', 320),
                    confidence_threshold=vision_config.get('confidence', 0.5)
                )
                if self.yolo_detector.net is None:
                    logger.warning("YOLO model not loaded - vision features disabled")
                    logger.warning(f"Model path: {model_path}")
                    logger.warning("You can continue without vision - basic commands will work")
                    self.yolo_detector = None
            except Exception as e:
                logger.warning(f"Failed to initialize YOLO detector: {e}")
                logger.warning("Continuing without vision features")
                self.yolo_detector = None
            
            # Initialize object tracker
            if self.yolo_detector:
                self.object_tracker = ObjectTracker(use_deepsort=True)
                
                # Initialize tracking controller
                tracking_config = self.config.get_tracking_config()
                tracking_config['camera_width'] = camera_config.get('width', 640)
                tracking_config['camera_height'] = camera_config.get('height', 480)
                tracking_config['fov_horizontal'] = camera_config.get('fov_horizontal', 120)
                tracking_config['fov_vertical'] = camera_config.get('fov_vertical', 90)
                tracking_config['pid_gains'] = self.config.get_pid_config()
                # Fixed 3m target distance (using forward ToF sensor)
                tracking_config['target_distance'] = tracking_config.get('target_distance', 3.0)
                tracking_config['target_altitude'] = tracking_config.get('altitude', {}).get('target_altitude', 1.5)
                tracking_config['lost_target_timeout'] = tracking_config.get('lost_target', {}).get('hover_timeout', 2.0)
                
                self.tracking_controller = TrackingController(tracking_config)
            
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
        
        # Add ToF reading loop if sensors are available
        if self.tof_sensors:
            tasks.append(self._tof_reading_loop())
        
        # Add vision loop if components are available
        if self.yolo_detector and self.object_tracker:
            tasks.append(self._vision_loop())
        
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
    
    async def _tof_reading_loop(self) -> None:
        """ToF sensor reading loop."""
        logger.info("ToF reading loop started")
        
        if not self.tof_sensors:
            logger.info("ToF reading loop disabled - sensors not available")
            while self.running:
                await asyncio.sleep(1)
            return
        
        while self.running:
            try:
                # Read all ToF sensors
                tof_data = self.tof_sensors.get_all_readings()
                
                # Update telemetry with ToF readings
                self.telemetry.update_from_tof(tof_data)
                
                await asyncio.sleep(0.1)  # 10 Hz reading rate
                
            except Exception as e:
                logger.error(f"Error in ToF reading loop: {e}")
                await asyncio.sleep(0.1)
    
    async def _vision_loop(self) -> None:
        """Vision processing and tracking loop."""
        logger.info("Vision loop started")
        
        # Skip if vision components not available
        if not self.yolo_detector or not self.object_tracker or not self.camera:
            logger.info("Vision loop disabled - components not available")
            while self.running:
                await asyncio.sleep(1)
            return
        
        while self.running:
            try:
                # Capture frame
                frame = self.camera.capture_frame()
                if frame is None:
                    await asyncio.sleep(0.1)
                    continue
                
                # Run YOLO detection
                detections = self.yolo_detector.detect(frame)
                
                # Update object tracker
                tracked_detections = self.object_tracker.update(detections, frame=frame)
                
                # Get current altitude for tracking controller
                height_agl = self.telemetry.get_height_agl()
                current_time = time.time()
                
                # Get forward ToF distance if available
                forward_tof_distance = None
                if self.tof_sensors:
                    forward_tof_distance = self.tof_sensors.get_forward_distance()
                
                # Update tracking controller and get control command
                if self.tracking_controller and self.tracking_controller.is_tracking():
                    control_command = self.tracking_controller.update(
                        tracked_detections, 
                        current_altitude=height_agl,
                        timestamp=current_time,
                        forward_tof_distance=forward_tof_distance
                    )
                    
                    if control_command:
                        # Send velocity command to drone
                        if self.drone_controller and self.drone_controller.is_connected():
                            # Ensure ALT_HOLD mode for following
                            if self.drone_controller.get_mode() != 'ALT_HOLD':
                                self.drone_controller.set_mode('ALT_HOLD')
                            
                            # Enforce height limit on vertical velocity
                            adjusted_vz = self._enforce_height_limit(control_command.vz)
                            
                            # Send velocity command
                            self.drone_controller.send_velocity_command(
                                vx=control_command.vx,
                                vy=control_command.vy,
                                vz=adjusted_vz,
                                yaw_rate=control_command.yaw_rate
                            )
                    else:
                        # Target lost - enter loiter mode
                        if self.drone_controller and self.drone_controller.is_connected():
                            logger.warning("Target lost - entering loiter mode")
                            self.drone_controller.set_mode_loiter()
                            self.tracking_controller.stop_tracking()
                
                # Send detections to backend
                if self.websocket and self.websocket.connected:
                    try:
                        await self.websocket.send_detections(tracked_detections, frame)
                    except Exception as e:
                        logger.error(f"Error sending detections: {e}")
                
                # Update tracking status in telemetry
                if self.tracking_controller:
                    if self.tracking_controller.is_tracking():
                        self.telemetry.update_tracking_status(f"TRACKING_ID_{self.tracking_controller.current_target_id}")
                    else:
                        self.telemetry.update_tracking_status("IDLE")
                
                await asyncio.sleep(0.05)  # 20 Hz
                
            except Exception as e:
                logger.error(f"Error in vision loop: {e}")
                import traceback
                logger.debug(traceback.format_exc())
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
        
        # Cleanup camera
        if self.camera:
            self.camera.cleanup()
        
        # Cleanup ToF sensors
        if self.tof_sensors:
            self.tof_sensors.cleanup()
        
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
                return await self._handle_follow(payload)
                
            elif command == 'stop_following':
                return await self._handle_stop_following(payload)
                
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
        
        # Limit to max 2m
        max_altitude = 2.0
        if takeoff_altitude > max_altitude:
            logger.warning(f"Requested altitude {takeoff_altitude}m exceeds max {max_altitude}m, limiting")
            takeoff_altitude = max_altitude
        
        # ToF sensors are mandatory for takeoff
        if not self.tof_sensors:
            return {'success': False, 'message': 'Cannot takeoff - ToF sensors not available. ToF sensors are required for safe flight.'}
        
        logger.info(f"Starting incremental throttle takeoff to {takeoff_altitude}m using bottom sensor")
        
        # Create callback function to get bottom distance
        def get_bottom_distance():
            return self.tof_sensors.get_bottom_distance()
        
        # Get ToF config
        tof_config = self.config.get_tof_config()
        ground_threshold = tof_config.get('ground_threshold', 0.08)  # 8cm default
        
        success = self.drone_controller.incremental_throttle_takeoff(
            get_bottom_distance=get_bottom_distance,
            target_altitude=takeoff_altitude,
            max_altitude=max_altitude,
            ground_threshold=ground_threshold
        )
        
        if success:
            return {'success': True, 'message': f'Takeoff to {takeoff_altitude}m started'}
        else:
            return {'success': False, 'message': 'Takeoff command failed'}
    
    async def _handle_land(self) -> Dict[str, Any]:
        """Handle land command."""
        if not self.drone_controller or not self.drone_controller.is_connected():
            return {'success': False, 'message': 'Drone controller not connected'}
        
        # ToF sensors are mandatory for landing
        if not self.tof_sensors:
            return {'success': False, 'message': 'Cannot land - ToF sensors not available. ToF sensors are required for safe landing.'}
        
        logger.info("Starting incremental throttle landing using bottom sensor")
        
        # Create callback function to get bottom distance
        def get_bottom_distance():
            return self.tof_sensors.get_bottom_distance()
        
        # Get ToF config
        tof_config = self.config.get_tof_config()
        ground_threshold = tof_config.get('ground_threshold', 0.08)  # 8cm default
        
        success = self.drone_controller.incremental_throttle_land(
            get_bottom_distance=get_bottom_distance,
            ground_threshold=ground_threshold
        )
        
        if success:
            return {'success': True, 'message': 'Landing sequence completed'}
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
    
    async def _handle_follow(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        """Handle follow command."""
        if not self.tracking_controller:
            return {'success': False, 'message': 'Tracking controller not initialized'}
        
        target_id = payload.get('target_id')
        if target_id is None:
            return {'success': False, 'message': 'target_id not provided'}
        
        # Set target in tracking controller
        success = self.tracking_controller.set_target(target_id)
        if success:
            # Ensure ALT_HOLD mode for following
            if self.drone_controller and self.drone_controller.is_connected():
                if self.drone_controller.get_mode() != 'ALT_HOLD':
                    self.drone_controller.set_mode('ALT_HOLD')
            
            return {'success': True, 'message': f'Following target ID {target_id}'}
        else:
            return {'success': False, 'message': 'Failed to set target'}
    
    async def _handle_stop_following(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        """Handle stop following command."""
        if not self.tracking_controller:
            return {'success': False, 'message': 'Tracking controller not initialized'}
        
        # Stop tracking
        self.tracking_controller.stop_tracking()
        
        # Enter loiter mode
        if self.drone_controller and self.drone_controller.is_connected():
            self.drone_controller.set_mode_loiter()
        
        return {'success': True, 'message': 'Stopped following'}
    
    async def _handle_prearm_checks(self) -> Dict[str, Any]:
        """Handle prearm checks command."""
        if not self.drone_controller or not self.drone_controller.is_connected():
            return {'success': False, 'message': 'Drone controller not connected'}
        
        checks = {}
        
        # Check ToF sensors (mandatory for flight)
        if self.tof_sensors and self.tof_sensors.initialized:
            # Try to read sensors to verify they're working
            forward_dist = self.tof_sensors.get_forward_distance()
            bottom_dist = self.tof_sensors.get_bottom_distance()
            checks['tof_sensors'] = {
                'forward_available': forward_dist is not None,
                'bottom_available': bottom_dist is not None,
                'ok': forward_dist is not None and bottom_dist is not None
            }
        else:
            checks['tof_sensors'] = {
                'ok': False,
                'message': 'ToF sensors not initialized - required for flight'
            }
        
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
        
        # ToF sensors are mandatory - must be ok for flight
        tof_ok = checks.get('tof_sensors', {}).get('ok', False)
        battery_ok = checks.get('battery', {}).get('ok', False) if checks.get('battery', {}).get('ok') is not None else True  # Optional if unavailable
        armed_ok = checks.get('armed', {}).get('ok', False)
        mode_ok = checks.get('mode', {}).get('ok', False)
        
        all_ok = tof_ok and battery_ok and armed_ok and mode_ok
        
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
