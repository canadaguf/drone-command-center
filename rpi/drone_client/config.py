"""
Configuration management for drone client.
Loads settings from YAML files and environment variables.
"""

import os
import yaml
from pathlib import Path
from typing import Dict, Any, Optional
import logging

logger = logging.getLogger(__name__)

class Config:
    """Configuration manager for drone client."""
    
    def __init__(self, config_path: Optional[str] = None):
        """Initialize configuration.
        
        Args:
            config_path: Path to config file. If None, uses default.
        """
        if config_path is None:
            config_path = Path(__file__).parent / "config" / "production.yaml"
        
        self.config_path = Path(config_path)
        self._config = self._load_config()
        
    def _load_config(self) -> Dict[str, Any]:
        """Load configuration from YAML file."""
        try:
            with open(self.config_path, 'r') as f:
                config = yaml.safe_load(f)
            logger.info(f"Loaded config from {self.config_path}")
            return config
        except FileNotFoundError:
            logger.warning(f"Config file not found: {self.config_path}")
            return self._get_default_config()
        except yaml.YAMLError as e:
            logger.error(f"Error parsing config file: {e}")
            return self._get_default_config()
    
    def _get_default_config(self) -> Dict[str, Any]:
        """Get default configuration."""
        return {
            "mavlink": {
                "connection": "/dev/ttyAMA0",
                "baud": 256000,
                "takeoff_altitude": 2.0
            },
            "backend": {
                "ws_url": "wss://drone-command-center.onrender.com/ws?client=drone",
                "reconnect_interval": 5
            },
            "vision": {
                "model_path": "/home/pi/models/yolo11n.onnx",
                "input_size": 320,
                "confidence": 0.5,
                "target_fps": 25
            },
            "camera": {
                "width": 640,
                "height": 480,
                "fps": 30,
                "fov_horizontal": 120,
                "fov_vertical": 90
            },
            "tracking": {
                "distance_modes": {
                    "close": 2.0,
                    "medium": 4.0,
                    "far": 6.0
                },
                "default_mode": "medium",
                "altitude": {
                    "auto_adjust": True,
                    "min_height": 1.0,
                    "max_height": 10.0
                },
                "lost_target": {
                    "hover_timeout": 2.0,
                    "land_timeout": 10.0
                }
            },
            "tof_sensors": {
                "multiplexer_address": 0x70,
                "sensors": [
                    {
                        "name": "forward",
                        "channel": 0,
                        "direction": "front",
                        "max_range": 4.0
                    },
                    {
                        "name": "down",
                        "channel": 1,
                        "direction": "down",
                        "max_range": 4.0
                    }
                ]
            },
            "safety": {
                "battery": {
                    "warn_percent": 30,
                    "critical_percent": 20,
                    "auto_land_percent": 15
                },
                "rc_override": {
                    "detection_enabled": True,
                    "threshold": 100
                },
                "geofence": {
                    "enabled": False,
                    "max_altitude": 50,
                    "max_distance": 100
                }
            },
            "pid_gains": {
                "yaw": {"kp": 0.5, "ki": 0.0, "kd": 0.1},
                "pitch": {"kp": 0.3, "ki": 0.0, "kd": 0.05},
                "roll": {"kp": 0.3, "ki": 0.0, "kd": 0.05},
                "distance": {"kp": 0.2, "ki": 0.0, "kd": 0.1}
            },
            "vlm": {
                "enabled": False,
                "provider": "gemini",
                "model": "gemini-2.0-flash",
                "api_key": "${GEMINI_API_KEY}"
            },
            "logging": {
                "level": "INFO",
                "file": "/var/log/drone_client.log",
                "max_size_mb": 50
            }
        }
    
    def get(self, key: str, default: Any = None) -> Any:
        """Get configuration value by key (supports dot notation)."""
        keys = key.split('.')
        value = self._config
        
        try:
            for k in keys:
                value = value[k]
            return value
        except (KeyError, TypeError):
            return default
    
    def set(self, key: str, value: Any) -> None:
        """Set configuration value by key (supports dot notation)."""
        keys = key.split('.')
        config = self._config
        
        # Navigate to parent of target key
        for k in keys[:-1]:
            if k not in config:
                config[k] = {}
            config = config[k]
        
        # Set the value
        config[keys[-1]] = value
    
    def get_mavlink_config(self) -> Dict[str, Any]:
        """Get MAVLink configuration."""
        return self.get('mavlink', {})
    
    def get_backend_config(self) -> Dict[str, Any]:
        """Get backend configuration."""
        return self.get('backend', {})
    
    def get_vision_config(self) -> Dict[str, Any]:
        """Get vision configuration."""
        return self.get('vision', {})
    
    def get_camera_config(self) -> Dict[str, Any]:
        """Get camera configuration."""
        return self.get('camera', {})
    
    def get_tracking_config(self) -> Dict[str, Any]:
        """Get tracking configuration."""
        return self.get('tracking', {})
    
    def get_tof_config(self) -> Dict[str, Any]:
        """Get TOF sensors configuration."""
        return self.get('tof_sensors', {})
    
    def get_safety_config(self) -> Dict[str, Any]:
        """Get safety configuration."""
        return self.get('safety', {})
    
    def get_pid_config(self) -> Dict[str, Any]:
        """Get PID configuration."""
        return self.get('pid_gains', {})
    
    def get_flight_mode_config(self) -> Dict[str, Any]:
        """Get flight mode configuration."""
        return self.get('flight_modes', {})
    
    def get_vlm_config(self) -> Dict[str, Any]:
        """Get VLM configuration."""
        return self.get('vlm', {})
    
    def get_logging_config(self) -> Dict[str, Any]:
        """Get logging configuration."""
        return self.get('logging', {})

# Global config instance
config = Config()
