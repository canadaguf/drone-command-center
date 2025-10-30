"""
Main orchestrator for autonomous drone client.
Coordinates all subsystems and manages async event loops.
"""

import asyncio
import logging
import signal
import sys
import time
from typing import Dict, Any, Optional
import numpy as np

# Import all components
from .config import Config
from .controllers.mavlink_controller import MAVLinkController
from .controllers.pid_controller import PIDManager
from .controllers.tracking_controller import TrackingController
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
        self.mavlink = None
        self.pid_manager = None
        self.tracking_controller = None
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
            
            # Initialize MAVLink controller
            mavlink_config = self.config.get_mavlink_config()
            self.mavlink = MAVLinkController(mavlink_config)
            if not self.mavlink.connect():
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
            
            # Initialize WebSocket client
            backend_config = self.config.get_backend_config()
            self.websocket = WebSocketClient(backend_config)
            
            # Initialize safety components
            safety_config = self.config.get_safety_config()
            self.rc_monitor = RCOverrideMonitor(safety_config.get('rc_override', {}))
            self.lost_target_handler = LostTargetHandler(tracking_config.get('lost_target', {}))
            self.battery_monitor = BatteryMonitor(safety_config.get('battery', {}))
            
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
                
                # Update tracking controller
                # Use default values if TOF sensors not available
                tof_forward = self.tof_manager.get_forward_distance() if self.tof_manager else None
                tof_down = self.tof_manager.get_down_distance() if self.tof_manager else None
                
                tracking_commands = self.tracking_controller.update(
                    tracked_persons, tof_forward, tof_down
                )
                
                # Send RC commands if tracking is active
                if tracking_commands.get('tracking_active', False):
                    self.mavlink.send_rc_override(
                        tracking_commands['roll'],
                        tracking_commands['pitch'],
                        tracking_commands['yaw'],
                        tracking_commands['throttle']
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
                # Get telemetry from MAVLink
                mavlink_telemetry = self.mavlink.get_telemetry()
                self.telemetry.update_from_mavlink(mavlink_telemetry)
                
                # Get TOF data (if available)
                if self.tof_manager:
                    tof_readings = self.tof_manager.get_all_readings()
                    self.telemetry.update_from_tof(tof_readings)
                
                # Update tracking status
                target_status = self.lost_target_handler.get_status_string()
                self.telemetry.update_tracking_status(target_status)
                
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
                telemetry = self.mavlink.get_telemetry()
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
                    self.mavlink.request_land()
                
                await asyncio.sleep(0.2)  # 5 Hz
                
            except Exception as e:
                logger.error(f"Error in safety loop: {e}")
                await asyncio.sleep(0.2)
    
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
        
        # Disconnect MAVLink
        if self.mavlink:
            self.mavlink.disconnect()
        
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
    
    def _on_command_received(self, command: str, payload: Dict[str, Any]) -> None:
        """Handle command received."""
        logger.info(f"Command received: {command}")
        
        try:
            if command == 'arm':
                success = self.mavlink.arm()
                if not success:
                    logger.error("Arm command failed - check MAVLink connection")
            elif command == 'disarm':
                success = self.mavlink.disarm()
                if not success:
                    logger.error("Disarm command failed - check MAVLink connection")
            elif command == 'takeoff':
                success = self.mavlink.request_takeoff(2.0)  # 2m altitude
                if not success:
                    logger.error("Takeoff command failed - check MAVLink connection")
            elif command == 'land':
                success = self.mavlink.request_land()
                if not success:
                    logger.error("Land command failed - check MAVLink connection")
            elif command == 'freeze':
                success = self.mavlink.set_mode('GUIDED')
                if not success:
                    logger.error("Freeze command failed - check MAVLink connection")
            elif command == 'follow':
                target_id = payload.get('target_id')
                if self.tracking_controller:
                    self.tracking_controller.set_target(target_id)
                else:
                    logger.error("Tracking controller not initialized")
            elif command == 'stop_following':
                if self.tracking_controller:
                    self.tracking_controller.stop_tracking()
                else:
                    logger.error("Tracking controller not initialized")
            elif command == 'set_distance_mode':
                mode = payload.get('mode')
                if self.tracking_controller:
                    self.tracking_controller.set_distance_mode(mode)
                else:
                    logger.error("Tracking controller not initialized")
        except Exception as e:
            logger.error(f"Error executing command {command}: {e}")
            import traceback
            logger.debug(traceback.format_exc())
    
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
        self.mavlink.set_mode('GUIDED')
    
    def _on_target_land(self) -> None:
        """Handle target land."""
        logger.warning("Target lost too long - initiating landing")
        self.mavlink.request_land()
    
    def _on_battery_warning(self, battery_percent: float) -> None:
        """Handle battery warning."""
        logger.warning(f"Battery warning: {battery_percent}%")
    
    def _on_battery_critical(self, battery_percent: float) -> None:
        """Handle battery critical."""
        logger.error(f"Battery critical: {battery_percent}%")
    
    def _on_battery_auto_land(self, battery_percent: float) -> None:
        """Handle battery auto-land."""
        logger.critical(f"Battery emergency - auto-land: {battery_percent}%")
        self.mavlink.request_land()

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
