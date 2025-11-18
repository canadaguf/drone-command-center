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
import numpy as np

# Import all components
from .config import Config
from .controllers.dronekit_controller import DroneKitController
from .controllers.pid_controller import PIDManager
from .controllers.tracking_controller import TrackingController
from .controllers.flight_mode_manager import FlightModeManager, FlightMode
from .vision.yolo_detector import YOLODetector
from .vision.person_tracker import PersonTracker
from .vision.depth_estimator import DepthEstimator
from .sensors.camera import CameraManager
from .sensors.tof_manager import TOFManager
from .sensors.telemetry import TelemetryCollector
from .communication.websocket_client import WebSocketClient
from .safety.rc_override_monitor import RCOverrideMonitor
from .safety.lost_target_handler import LostTargetHandler, TargetStatus
from .safety.battery_monitor import BatteryMonitor, BatteryStatus

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
        
        # Initialize components
        self.dronekit = None
        self.pid_manager = None
        self.tracking_controller = None
        self.flight_mode_manager = None
        self.yolo_detector = None
        self.person_tracker = None
        self.depth_estimator = None
        self.camera = None
        self.tof_manager = None
        self.telemetry = None
        self.websocket = None
        self.rc_monitor = None
        self.lost_target_handler = None
        self.battery_monitor = None
        
        # State
        self.running = False
        self.initialized = False
        
        # Event loops
        self.vision_loop_task = None
        self.telemetry_loop_task = None
        self.command_loop_task = None
        self.safety_loop_task = None
        self.flight_mode_loop_task = None
        
        # Setup signal handlers
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
    
    def _signal_handler(self, signum, frame):
        """Handle shutdown signals."""
        logger.info(f"Received signal {signum} (Ctrl+C), shutting down gracefully...")
        self.running = False
        # Actual shutdown will happen in start() method when running=False
    
    async def initialize(self) -> bool:
        """Initialize all components.
        
        Returns:
            True if initialization successful, False otherwise
        """
        try:
            logger.info("Initializing drone client...")
            
            # Initialize DroneKit controller
            mavlink_config = self.config.get_mavlink_config()
            self.dronekit = DroneKitController(mavlink_config)
            if not self.dronekit.connect():
                logger.error("Failed to connect to flight controller")
                return False
            
            # Initialize PID manager
            pid_config = self.config.get_pid_config()
            self.pid_manager = PIDManager(pid_config)
            
            # Initialize vision components (optional - can work without YOLO)
            vision_config = self.config.get_vision_config()
            model_path = vision_config.get('model_path', '/home/pi/models/yolo11n.onnx')
            
            # Try to load YOLO detector, but don't fail if model not available
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
            except Exception as e:
                logger.warning(f"Failed to initialize YOLO detector: {e}")
                logger.warning("Continuing without vision features")
                self.yolo_detector = None
            
            self.person_tracker = PersonTracker()
            
            camera_config = self.config.get_camera_config()
            self.depth_estimator = DepthEstimator(camera_config)
            
            # Initialize camera
            self.camera = CameraManager(camera_config)
            if not self.camera.initialize():
                logger.error("Failed to initialize camera")
                return False
            
            # Initialize TOF manager (optional - can work without sensors)
            tof_config = self.config.get_tof_config()
            try:
                self.tof_manager = TOFManager(tof_config)
                if not self.tof_manager.start_reading():
                    logger.warning("Failed to start TOF reading - continuing without TOF sensors")
                    logger.warning("TOF sensors are optional - system will work without them")
                    self.tof_manager = None
            except Exception as e:
                logger.warning(f"TOF sensors not available: {e}")
                logger.warning("Continuing without TOF sensors - they are optional")
                self.tof_manager = None
            
            # Initialize telemetry
            self.telemetry = TelemetryCollector()
            
            # Initialize tracking controller
            tracking_config = self.config.get_tracking_config()
            self.tracking_controller = TrackingController(
                tracking_config, self.pid_manager, self.telemetry
            )
            # Initialize distance mode in telemetry
            default_mode = tracking_config.get('default_mode', 'medium')
            self.telemetry.update_distance_mode(default_mode)
            
            # Initialize WebSocket client
            backend_config = self.config.get_backend_config()
            self.websocket = WebSocketClient(backend_config)
            
            # Initialize safety components
            safety_config = self.config.get_safety_config()
            self.rc_monitor = RCOverrideMonitor(safety_config.get('rc_override', {}))
            self.lost_target_handler = LostTargetHandler(tracking_config.get('lost_target', {}))
            self.battery_monitor = BatteryMonitor(safety_config.get('battery', {}))
            
            # Initialize flight mode manager
            flight_mode_config = self.config.get_flight_mode_config()
            self.flight_mode_manager = FlightModeManager(flight_mode_config)
            self.flight_mode_manager.set_callbacks(
                on_mode_change=self._on_flight_mode_change,
                on_landed=self._on_landed
            )
            
            # Setup callbacks
            self._setup_callbacks()
            
            self.initialized = True
            logger.info("Drone client initialized successfully")
            return True
            
        except Exception as e:
            logger.error(f"Failed to initialize drone client: {e}")
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
        
        # RC override callbacks
        self.rc_monitor.set_callbacks(
            on_start=self._on_rc_override_start,
            on_end=self._on_rc_override_end
        )
        
        # Lost target callbacks
        self.lost_target_handler.set_callbacks(
            on_status_change=self._on_target_status_change,
            on_hover=self._on_target_hover,
            on_land=self._on_target_land
        )
        
        # Battery callbacks
        self.battery_monitor.set_callbacks(
            on_warning=self._on_battery_warning,
            on_critical=self._on_battery_critical,
            on_auto_land=self._on_battery_auto_land
        )
    
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
            self._vision_loop(),
            self._telemetry_loop(),
            self._command_loop(),
            self._safety_loop(),
            self._flight_mode_loop(),
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
    
    async def _vision_loop(self) -> None:
        """Vision processing loop."""
        logger.info("Vision loop started")
        
        # Skip vision loop if YOLO detector not available
        if self.yolo_detector is None or self.yolo_detector.net is None:
            logger.info("Vision loop disabled - YOLO model not available")
            while self.running:
                await asyncio.sleep(1)  # Sleep longer when disabled
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
                
                # Update person tracker
                tracked_persons = self.person_tracker.update(detections)
                
                # Update tracking controller (visual-only, no sensors)
                # Get height AGL from telemetry collector
                height_agl = self.telemetry.get_height_agl()  # Height above ground level
                
                # Tracking uses only visual data - no ToF sensors
                tracking_commands = self.tracking_controller.update(
                    tracked_persons, height_agl
                )
                
                # Send RC commands ONLY if tracking is active
                # When not tracking, send stabilization commands (zero roll/pitch, hover throttle)
                if tracking_commands.get('tracking_active', False):
                    self.dronekit.send_rc_override(
                        tracking_commands['roll'],
                        tracking_commands['pitch'],
                        tracking_commands['yaw'],
                        tracking_commands['throttle']
                    )
                else:
                    # Not tracking - send stabilization commands to prevent falling
                    # Always maintain hover throttle and zero roll/pitch for stability
                    if self.flight_mode_manager:
                        stabilization = self.flight_mode_manager.get_stabilization_commands()
                        self.dronekit.send_rc_override(
                            stabilization['roll'],
                            stabilization['pitch'],
                            stabilization['yaw'],
                            stabilization['throttle']
                        )
                
                # Send detections to backend
                await self.websocket.send_detections(tracked_persons, frame)
                
                # Update lost target handler
                target_detected = len(tracked_persons) > 0
                self.lost_target_handler.update(target_detected)
                
                await asyncio.sleep(0.05)  # 20 Hz
                
            except Exception as e:
                logger.error(f"Error in vision loop: {e}")
                await asyncio.sleep(0.1)
    
    async def _telemetry_loop(self) -> None:
        """Telemetry streaming loop."""
        logger.info("Telemetry loop started")
        
        while self.running:
            try:
                # Get telemetry from DroneKit
                dronekit_telemetry = self.dronekit.get_telemetry()
                self.telemetry.update_from_mavlink(dronekit_telemetry)  # Compatible interface
                
                # Get TOF data (if available)
                if self.tof_manager:
                    tof_readings = self.tof_manager.get_all_readings()
                    self.telemetry.update_from_tof(tof_readings)
                
                # Update tracking status
                target_status = self.lost_target_handler.get_status_string()
                self.telemetry.update_tracking_status(target_status)
                
                # Update distance mode from tracking controller
                if self.tracking_controller:
                    tracking_status = self.tracking_controller.get_status()
                    self.telemetry.update_distance_mode(tracking_status.get('distance_mode'))
                
                # Get telemetry data
                telemetry_data = self.telemetry.get_telemetry()
                
                # Send to backend
                await self.websocket.send_telemetry(telemetry_data)
                
                await asyncio.sleep(0.5)  # 2 Hz
                
            except Exception as e:
                logger.error(f"Error in telemetry loop: {e}")
                await asyncio.sleep(0.5)
    
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
    
    async def _safety_loop(self) -> None:
        """Safety monitoring loop."""
        logger.info("Safety loop started")
        
        while self.running:
            try:
                # Check RC override
                telemetry = self.dronekit.get_telemetry()
                rc_channels = {
                    'roll': telemetry.get('roll', 1500),
                    'pitch': telemetry.get('pitch', 1500),
                    'yaw': telemetry.get('yaw', 1500),
                    'throttle': telemetry.get('throttle', 1500)
                }
                self.rc_monitor.update(rc_channels)
                
                # Check battery
                battery_percent = telemetry.get('battery_remaining')
                battery_voltage = telemetry.get('battery_voltage')
                self.battery_monitor.update(battery_percent, battery_voltage)
                
                # Check for safety actions
                if self.battery_monitor.is_auto_land_triggered():
                    logger.critical("Battery auto-land triggered")
                    # Use flight mode manager for gradual landing
                    if self.flight_mode_manager:
                        self.flight_mode_manager.set_mode(FlightMode.LANDING)
                
                await asyncio.sleep(0.2)  # 5 Hz
                
            except Exception as e:
                logger.error(f"Error in safety loop: {e}")
                await asyncio.sleep(0.2)
    
    async def _flight_mode_loop(self) -> None:
        """Flight mode state management loop."""
        logger.info("Flight mode loop started")
        
        if not self.flight_mode_manager:
            logger.error("Flight mode manager not initialized")
            return
        
        while self.running:
            try:
                # Get current flight mode
                current_mode = self.flight_mode_manager.get_mode()
                
                # Get ToF sensor readings (fallback)
                tof_down = self.tof_manager.get_down_distance() if self.tof_manager else None
                
                # Get height AGL from telemetry collector (primary)
                height_agl = self.telemetry.get_height_agl()
                
                # Handle different flight modes
                if current_mode == FlightMode.TAKING_OFF:
                    # DroneKit handles takeoff automatically, just monitor progress
                    # Check if target altitude reached
                    target_altitude = self.config.get_mavlink_config().get('takeoff_altitude', 1.5)
                    if height_agl is not None and height_agl >= target_altitude * 0.9:  # 90% of target
                        logger.info("Takeoff complete - entering flying mode")
                        self.flight_mode_manager.set_mode(FlightMode.FLYING)
                
                elif current_mode == FlightMode.LANDING:
                    # DroneKit handles landing automatically, just monitor progress
                    # Check if landed
                    if height_agl is not None and height_agl <= 0.1:  # Within 10cm of ground
                        logger.info("Landing detected - disarming motors")
                        self.dronekit.disarm()
                        # Reset altitude reference after landing
                        self.telemetry.reset_altitude_reference()
                        self.flight_mode_manager.set_mode(FlightMode.LANDED)
                
                elif current_mode == FlightMode.LOITERING:
                    # Loiter mode is handled by flight controller, no RC override needed
                    pass
                
                elif current_mode in [FlightMode.FLYING, FlightMode.IDLE]:
                    # In flying/idle mode, stabilization is handled by vision loop
                    # or flight controller's built-in modes
                    pass
                
                await asyncio.sleep(0.1)  # 10 Hz for flight mode updates
                
            except Exception as e:
                logger.error(f"Error in flight mode loop: {e}")
                await asyncio.sleep(0.1)
    
    async def shutdown(self) -> None:
        """Shutdown drone client."""
        logger.info("Shutting down drone client...")
        
        self.running = False
        
        # Stop TOF reading (if initialized)
        if self.tof_manager:
            try:
                self.tof_manager.stop_reading()
            except Exception as e:
                logger.warning(f"Error stopping TOF reading: {e}")
        
        # Disconnect WebSocket
        if self.websocket:
            await self.websocket.disconnect()
        
        # Disconnect DroneKit
        if self.dronekit:
            self.dronekit.disconnect()
        
        # Cleanup camera
        if self.camera:
            self.camera.cleanup()
        
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
        
        Returns:
            Dictionary with 'success' (bool) and 'message' (str) keys
        """
        logger.info(f"Command received: {command}")
        
        try:
            if command == 'arm':
                # Use DroneKit for arming
                success = self.dronekit.arm()
                if not success:
                    return {'success': False, 'message': 'Arm command failed - check connection'}
                
                # Capture altitude reference when arming
                dronekit_telemetry = self.dronekit.get_telemetry()
                self.telemetry.update_from_mavlink(dronekit_telemetry)
                if not self.telemetry.altitude_reference_captured:
                    self.telemetry.capture_altitude_reference()
                    logger.info("Altitude reference captured at arming")
                
                if self.flight_mode_manager:
                    self.flight_mode_manager.set_mode(FlightMode.ARMING)
                return {'success': True, 'message': 'Drone armed successfully', 'armed': True}
                    
            elif command == 'disarm':
                # Only disarm if landed (ToF < 6.6cm) or if explicitly requested
                tof_down = self.tof_manager.get_down_distance() if self.tof_manager else None
                if tof_down is not None:
                    landing_max = 0.06 + 0.006  # 6.6cm
                    if tof_down > landing_max:
                        logger.warning(f"Disarm requested but drone not landed (ToF: {tof_down:.3f}m)")
                        logger.warning("Disarm will proceed anyway - use with caution")
                
                success = self.dronekit.disarm()
                if not success:
                    return {'success': False, 'message': 'Disarm command failed - check connection'}
                
                # Reset altitude reference on disarm
                self.telemetry.reset_altitude_reference()
                logger.info("Altitude reference reset after disarm")
                if self.flight_mode_manager:
                    self.flight_mode_manager.set_mode(FlightMode.IDLE)
                return {'success': True, 'message': 'Drone disarmed successfully', 'armed': False}
                
            elif command == 'takeoff':
                # Ensure altitude reference is captured before takeoff
                if not self.telemetry.altitude_reference_captured:
                    dronekit_telemetry = self.dronekit.get_telemetry()
                    self.telemetry.update_from_mavlink(dronekit_telemetry)
                    if not self.telemetry.capture_altitude_reference():
                        logger.warning("Failed to capture altitude reference - takeoff may be inaccurate")
                
                # Get takeoff altitude from config
                takeoff_altitude = self.config.get_mavlink_config().get('takeoff_altitude', 1.5)
                
                # Use DroneKit simple_takeoff
                logger.info(f"Starting takeoff to {takeoff_altitude}m using DroneKit")
                success = self.dronekit.simple_takeoff(takeoff_altitude)
                
                if success:
                    if self.flight_mode_manager:
                        self.flight_mode_manager.set_mode(FlightMode.TAKING_OFF)
                    return {'success': True, 'message': f'Takeoff to {takeoff_altitude}m started'}
                else:
                    return {'success': False, 'message': 'Takeoff command failed'}
                    
            elif command == 'land':
                # Use DroneKit simple_land
                logger.info("Starting landing sequence using DroneKit")
                success = self.dronekit.simple_land()
                
                if success:
                    if self.flight_mode_manager:
                        self.flight_mode_manager.set_mode(FlightMode.LANDING)
                    return {'success': True, 'message': 'Landing sequence started'}
                else:
                    return {'success': False, 'message': 'Landing command failed'}
                    
            elif command == 'freeze':
                # Set LOITER mode using DroneKit
                success = self.dronekit.set_mode_loiter()
                if success:
                    if self.flight_mode_manager:
                        self.flight_mode_manager.set_mode(FlightMode.LOITERING)
                    return {'success': True, 'message': 'Loiter mode activated'}
                else:
                    return {'success': False, 'message': 'Loiter command failed'}
                    
            elif command == 'follow':
                target_id = payload.get('target_id')
                if self.tracking_controller:
                    self.tracking_controller.set_target(target_id)
                    return {'success': True, 'message': f'Following target {target_id}'}
                else:
                    return {'success': False, 'message': 'Tracking controller not initialized'}
                    
            elif command == 'stop_following':
                if self.tracking_controller:
                    self.tracking_controller.stop_tracking()
                    # Clear RC override to release control back to flight controller
                    self.dronekit.clear_rc_override()
                    # Set GUIDED mode for position hold when tracking stops
                    self.dronekit.set_mode('GUIDED')
                    logger.info("Stop following - cleared RC override and switched to GUIDED mode")
                    return {'success': True, 'message': 'Stopped following'}
                else:
                    return {'success': False, 'message': 'Tracking controller not initialized'}
            elif command == 'set_distance_mode':
                mode = payload.get('mode')
                if self.tracking_controller:
                    success = self.tracking_controller.set_distance_mode(mode)
                    if success:
                        # Update telemetry with new mode
                        self.telemetry.update_distance_mode(mode)
                        logger.info(f"Distance mode changed to {mode}")
                        return {'success': True, 'message': f'Distance mode set to {mode}'}
                    else:
                        logger.error(f"Failed to set distance mode to {mode}")
                        return {'success': False, 'message': f'Failed to set distance mode to {mode}'}
                else:
                    logger.error("Tracking controller not initialized")
                    return {'success': False, 'message': 'Tracking controller not initialized'}
            else:
                return {'success': False, 'message': f'Unknown command: {command}'}
        except Exception as e:
            logger.error(f"Error executing command {command}: {e}")
            import traceback
            logger.debug(traceback.format_exc())
            return {'success': False, 'message': f'Error executing command: {str(e)}'}
    
    def _on_rc_override_start(self) -> None:
        """Handle RC override start."""
        logger.info("RC override started - manual control active")
        self.tracking_controller.stop_tracking()
    
    def _on_rc_override_end(self) -> None:
        """Handle RC override end."""
        logger.info("RC override ended - returning to autonomous")
    
    def _on_target_status_change(self, old_status: TargetStatus, new_status: TargetStatus) -> None:
        """Handle target status change."""
        logger.info(f"Target status changed: {old_status.value} → {new_status.value}")
    
    def _on_target_hover(self) -> None:
        """Handle target hover."""
        logger.info("Target lost - entering hover mode")
        self.dronekit.set_mode('GUIDED')
    
    def _on_target_land(self) -> None:
        """Handle target land."""
        logger.warning("Target lost too long - initiating landing")
        # Use DroneKit for landing
        if self.flight_mode_manager:
            self.flight_mode_manager.set_mode(FlightMode.LANDING)
        self.dronekit.simple_land()
    
    def _on_flight_mode_change(self, old_mode: FlightMode, new_mode: FlightMode) -> None:
        """Handle flight mode change."""
        logger.info(f"Flight mode changed: {old_mode.value} -> {new_mode.value}")
    
    def _on_landed(self) -> None:
        """Handle landing detection."""
        logger.info("Landing detected - disarming motors")
        self.dronekit.disarm()
        # Reset altitude reference after landing
        self.telemetry.reset_altitude_reference()
        logger.info("Altitude reference reset after landing")
    
    def _on_battery_warning(self, battery_percent: float) -> None:
        """Handle battery warning."""
        logger.warning(f"Battery warning: {battery_percent}%")
    
    def _on_battery_critical(self, battery_percent: float) -> None:
        """Handle battery critical."""
        logger.error(f"Battery critical: {battery_percent}%")
    
    def _on_battery_auto_land(self, battery_percent: float) -> None:
        """Handle battery auto-land."""
        logger.critical(f"Battery emergency - auto-land: {battery_percent}%")
        self.dronekit.simple_land()

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
