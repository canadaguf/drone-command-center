"""
WebSocket client for backend communication.
Handles connection to Render backend, command processing, and telemetry streaming.
Based on test_snippets/worked_best/drone_client.py
"""

import asyncio
import json
import logging
import websockets
from typing import Dict, Any, Optional, Callable, List
import base64
import cv2
import numpy as np

logger = logging.getLogger(__name__)

class WebSocketClient:
    """WebSocket client for backend communication."""
    
    def __init__(self, config: Dict[str, Any]):
        """Initialize WebSocket client.
        
        Args:
            config: Backend configuration
        """
        self.ws_url = config.get('ws_url', 'wss://drone-command-center.onrender.com/ws?client=drone')
        self.reconnect_interval = config.get('reconnect_interval', 5)
        
        # Connection state
        self.websocket = None
        self.connected = False
        self.running = False
        
        # Command handlers
        self.command_handlers = {}
        self._register_default_handlers()
        
        # Telemetry streaming
        self.telemetry_interval = 0.5  # 2 Hz
        self.last_telemetry_send = 0
        
        # Detection streaming
        self.detection_interval = 1.0  # 1 Hz
        self.last_detection_send = 0
        
        # Callbacks
        self.on_connect_callback = None
        self.on_disconnect_callback = None
        self.on_command_callback = None
        
        # Command executor (set by drone client)
        self.command_executor = None
        
    def _register_default_handlers(self) -> None:
        """Register default command handlers."""
        self.command_handlers = {
            'arm': self._handle_arm,
            'disarm': self._handle_disarm,
            'takeoff': self._handle_takeoff,
            'land': self._handle_land,
            'freeze': self._handle_freeze,
            'check_connection': self._handle_check_connection,
            'prearm_checks': self._handle_prearm_checks,
            'follow': self._handle_follow,
            'stop_following': self._handle_stop_following,
            'set_distance_mode': self._handle_set_distance_mode
        }
    
    def set_command_handler(self, command: str, handler: Callable) -> None:
        """Set custom command handler.
        
        Args:
            command: Command name
            handler: Handler function
        """
        self.command_handlers[command] = handler
        logger.info(f"Custom handler registered for command: {command}")
    
    def set_command_executor(self, executor: Callable) -> None:
        """Set command executor function.
        
        Args:
            executor: Function that executes commands (command_name, payload) -> None
        """
        self.command_executor = executor
        logger.info("Command executor set")
    
    def set_callbacks(self, on_connect: Optional[Callable] = None,
                     on_disconnect: Optional[Callable] = None,
                     on_command: Optional[Callable] = None) -> None:
        """Set callback functions.
        
        Args:
            on_connect: Called when connected
            on_disconnect: Called when disconnected
            on_command: Called when command received
        """
        self.on_connect_callback = on_connect
        self.on_disconnect_callback = on_disconnect
        self.on_command_callback = on_command
    
    async def connect(self) -> bool:
        """Connect to backend.
        
        Returns:
            True if connection successful, False otherwise
        """
        try:
            logger.info(f"Connecting to backend: {self.ws_url}")
            self.websocket = await websockets.connect(self.ws_url)
            self.connected = True
            self.running = True
            
            if self.on_connect_callback:
                self.on_connect_callback()
            
            logger.info("Connected to backend")
            return True
            
        except Exception as e:
            logger.error(f"Failed to connect to backend: {e}")
            self.connected = False
            return False
    
    async def disconnect(self) -> None:
        """Disconnect from backend."""
        self.running = False
        if self.websocket:
            await self.websocket.close()
            self.websocket = None
        self.connected = False
        
        if self.on_disconnect_callback:
            self.on_disconnect_callback()
        
        logger.info("Disconnected from backend")
    
    async def start(self) -> None:
        """Start WebSocket client."""
        while self.running:
            try:
                if not self.connected:
                    if not await self.connect():
                        logger.warning(f"Reconnecting in {self.reconnect_interval} seconds...")
                        await asyncio.sleep(self.reconnect_interval)
                        continue
                
                # Start message processing
                await self._message_loop()
                
            except websockets.exceptions.ConnectionClosed:
                logger.warning("WebSocket connection closed")
                self.connected = False
            except Exception as e:
                logger.error(f"Error in WebSocket client: {e}")
                self.connected = False
                await asyncio.sleep(self.reconnect_interval)
    
    async def _message_loop(self) -> None:
        """Main message processing loop."""
        try:
            async for message in self.websocket:
                await self._process_message(message)
        except websockets.exceptions.ConnectionClosed:
            raise
        except Exception as e:
            logger.error(f"Error in message loop: {e}")
            raise
    
    async def _process_message(self, message: str) -> None:
        """Process incoming message.
        
        Args:
            message: Raw message string
        """
        try:
            data = json.loads(message)
            logger.debug(f"Received message: {data}")
            
            # Check if command from web client
            if (data.get('source') == 'web' and 
                data.get('type') == 'command'):
                
                payload = data.get('payload', {})
                action = payload.get('action')
                
                if action in self.command_handlers:
                    # Execute command via command executor if set (async)
                    result = None
                    if self.command_executor:
                        # Command executor is async and returns result dict
                        result = await self.command_executor(action, payload)
                    else:
                        # Fallback to handler if no executor set
                        result = await self.command_handlers[action](payload)
                    
                    # Send acknowledgment with actual result
                    if result:
                        # Use result from command executor (has success/failure info)
                        await self._send_acknowledgment(action, result)
                    else:
                        # Fallback acknowledgment
                        await self._send_acknowledgment(action, {'message': f'{action} command executed'})
                    
                    if self.on_command_callback:
                        self.on_command_callback(action, payload)
                else:
                    logger.warning(f"Unknown command: {action}")
                    await self._send_error(f"Unknown command: {action}")
            
        except json.JSONDecodeError:
            logger.error(f"Invalid JSON received: {message}")
        except Exception as e:
            logger.error(f"Error processing message: {e}")
    
    async def _send_acknowledgment(self, action: str, result: Dict[str, Any]) -> None:
        """Send command acknowledgment.
        
        Args:
            action: Command action
            result: Command result (should have 'success' and 'message' keys)
        """
        # Determine message type based on success status
        success = result.get('success', True)  # Default to success if not specified
        msg_type = f'{action}_success' if success else f'{action}_error'
        
        message = {
            'source': 'drone',
            'type': msg_type,
            'payload': result
        }
        await self._send_message(message)
    
    async def _send_error(self, error_message: str) -> None:
        """Send error message.
        
        Args:
            error_message: Error message
        """
        message = {
            'source': 'drone',
            'type': 'error',
            'payload': {'message': error_message}
        }
        await self._send_message(message)
    
    async def send_telemetry(self, telemetry: Dict[str, Any]) -> None:
        """Send telemetry data.
        
        Args:
            telemetry: Telemetry data dictionary
        """
        message = {
            'source': 'drone',
            'type': 'telemetry',
            'payload': telemetry
        }
        await self._send_message(message)
    
    async def send_detections(self, detections: List[Dict[str, Any]], 
                             frame: Optional[np.ndarray] = None) -> None:
        """Send person detections.
        
        Args:
            detections: List of person detections
            frame: Optional frame for thumbnails
        """
        # Create detection payload
        persons = []
        for detection in detections:
            person_data = {
                'id': detection.get('id'),
                'class_name': detection.get('class_name'),
                'confidence': detection.get('confidence'),
                'bbox': detection.get('bbox'),
                'center': detection.get('center')
            }
            
            # Add thumbnail if frame provided
            if frame is not None and 'bbox' in detection:
                thumbnail = self._extract_thumbnail(frame, detection['bbox'])
                if thumbnail:
                    person_data['thumbnail'] = thumbnail
            
            persons.append(person_data)
        
        message = {
            'source': 'drone',
            'type': 'detections',
            'payload': {'persons': persons}
        }
        await self._send_message(message)
    
    def _extract_thumbnail(self, frame: np.ndarray, bbox: tuple) -> Optional[str]:
        """Extract thumbnail from frame.
        
        Args:
            frame: Input frame
            bbox: Bounding box (x1, y1, x2, y2)
            
        Returns:
            Base64 encoded thumbnail, None if extraction failed
        """
        try:
            x1, y1, x2, y2 = bbox
            
            # Ensure coordinates are within frame bounds
            x1 = max(0, min(x1, frame.shape[1] - 1))
            y1 = max(0, min(y1, frame.shape[0] - 1))
            x2 = max(x1 + 1, min(x2, frame.shape[1]))
            y2 = max(y1 + 1, min(y2, frame.shape[0]))
            
            # Extract region
            thumbnail = frame[y1:y2, x1:x2]
            
            if thumbnail.size == 0:
                return None
            
            # Resize to standard size
            thumbnail = cv2.resize(thumbnail, (80, 120))
            
            # Encode as JPEG
            _, buffer = cv2.imencode('.jpg', thumbnail, 
                                   [cv2.IMWRITE_JPEG_QUALITY, 80])
            
            # Convert to base64
            thumbnail_b64 = base64.b64encode(buffer).decode()
            return thumbnail_b64
            
        except Exception as e:
            logger.error(f"Failed to extract thumbnail: {e}")
            return None
    
    async def _send_message(self, message: Dict[str, Any]) -> None:
        """Send message to backend.
        
        Args:
            message: Message dictionary
        """
        if not self.connected or not self.websocket:
            logger.warning("Not connected, cannot send message")
            return
        
        try:
            await self.websocket.send(json.dumps(message))
            logger.debug(f"Sent message: {message['type']}")
        except Exception as e:
            logger.error(f"Failed to send message: {e}")
    
    # Command handlers
    async def _handle_arm(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        """Handle arm command."""
        logger.info("Arm command received")
        return {'message': 'Arm command executed'}
    
    async def _handle_disarm(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        """Handle disarm command."""
        logger.info("Disarm command received")
        return {'message': 'Disarm command executed'}
    
    async def _handle_takeoff(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        """Handle takeoff command."""
        logger.info("Takeoff command received")
        return {'message': 'Takeoff command executed'}
    
    async def _handle_land(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        """Handle land command."""
        logger.info("Land command received")
        return {'message': 'Land command executed'}
    
    async def _handle_freeze(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        """Handle freeze command."""
        logger.info("Freeze command received")
        return {'message': 'Freeze command executed'}
    
    async def _handle_check_connection(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        """Handle check connection command."""
        logger.info("Check connection command received")
        return {'message': 'Connection check executed'}
    
    async def _handle_prearm_checks(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        """Handle prearm checks command."""
        logger.info("Prearm checks command received")
        return {'message': 'Prearm checks executed'}
    
    async def _handle_follow(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        """Handle follow command."""
        target_id = payload.get('target_id')
        logger.info(f"Follow command received for person {target_id}")
        return {'message': f'Follow command executed for person {target_id}'}
    
    async def _handle_stop_following(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        """Handle stop following command."""
        target_id = payload.get('target_id')
        logger.info(f"Stop following command received for person {target_id}")
        return {'message': f'Stop following command executed for person {target_id}'}
    
    async def _handle_set_distance_mode(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        """Handle set distance mode command."""
        mode = payload.get('mode')
        logger.info(f"Set distance mode command received: {mode}")
        return {'message': f'Distance mode set to {mode}'}
    
    def get_status(self) -> Dict[str, Any]:
        """Get WebSocket client status.
        
        Returns:
            Dictionary containing status information
        """
        return {
            'connected': self.connected,
            'running': self.running,
            'ws_url': self.ws_url,
            'reconnect_interval': self.reconnect_interval,
            'registered_handlers': list(self.command_handlers.keys())
        }
