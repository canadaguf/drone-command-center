"""
Depth estimator for distance calculation from bounding box size.
Used when TOF sensors are not available or out of range.
"""

import numpy as np
import logging
from typing import Dict, Any, Tuple, Optional

logger = logging.getLogger(__name__)

class DepthEstimator:
    """Depth estimator using bounding box size heuristics."""
    
    def __init__(self, config: Dict[str, Any]):
        """Initialize depth estimator.
        
        Args:
            config: Configuration dictionary
        """
        # Camera parameters
        self.camera_width = config.get('width', 640)
        self.camera_height = config.get('height', 480)
        self.fov_horizontal = config.get('fov_horizontal', 120)  # degrees
        self.fov_vertical = config.get('fov_vertical', 90)  # degrees
        
        # Calibration parameters (empirically determined)
        self.person_height_meters = 1.7  # Average person height
        self.person_width_meters = 0.5  # Average person width
        
        # Distance estimation parameters
        self.min_distance = 1.0  # meters
        self.max_distance = 20.0  # meters
        self.fallback_distance = 5.0  # meters when estimation fails
        
        # Calibration data (distance -> bbox_height_ratio)
        self.calibration_data = {
            1.0: 0.8,   # 1m -> 80% of image height
            2.0: 0.4,   # 2m -> 40% of image height
            3.0: 0.27,  # 3m -> 27% of image height
            4.0: 0.2,   # 4m -> 20% of image height
            5.0: 0.16,  # 5m -> 16% of image height
            10.0: 0.08, # 10m -> 8% of image height
            20.0: 0.04  # 20m -> 4% of image height
        }
        
        logger.info(f"Depth estimator initialized: {self.camera_width}x{self.camera_height}, FOV={self.fov_horizontal}°x{self.fov_vertical}°")
    
    def estimate_distance(self, bbox: Tuple[int, int, int, int], 
                         tof_distance: Optional[float] = None,
                         max_tof_range: float = 4.0) -> float:
        """Estimate distance to person.
        
        Args:
            bbox: Bounding box (x1, y1, x2, y2)
            tof_distance: TOF sensor distance (if available)
            max_tof_range: Maximum reliable TOF range
            
        Returns:
            Estimated distance in meters
        """
        # Use TOF if available and within range
        if tof_distance is not None and 0.5 <= tof_distance <= max_tof_range:
            logger.debug(f"Using TOF distance: {tof_distance}m")
            return tof_distance
        
        # Estimate from bbox size
        estimated_distance = self._estimate_from_bbox(bbox)
        
        # Apply bounds
        estimated_distance = max(self.min_distance, min(self.max_distance, estimated_distance))
        
        logger.debug(f"Estimated distance from bbox: {estimated_distance}m")
        return estimated_distance
    
    def _estimate_from_bbox(self, bbox: Tuple[int, int, int, int]) -> float:
        """Estimate distance from bounding box size.
        
        Args:
            bbox: Bounding box (x1, y1, x2, y2)
            
        Returns:
            Estimated distance in meters
        """
        x1, y1, x2, y2 = bbox
        bbox_width = x2 - x1
        bbox_height = y2 - y1
        
        # Calculate bbox height ratio (0.0 to 1.0)
        height_ratio = bbox_height / self.camera_height
        
        # Use height-based estimation (more reliable than width)
        distance = self._height_ratio_to_distance(height_ratio)
        
        # Validate with width if available
        if bbox_width > 0:
            width_ratio = bbox_width / self.camera_width
            width_distance = self._width_ratio_to_distance(width_ratio)
            
            # Use average of height and width estimates
            distance = (distance + width_distance) / 2
        
        return distance
    
    def _height_ratio_to_distance(self, height_ratio: float) -> float:
        """Convert height ratio to distance.
        
        Args:
            height_ratio: Bbox height / image height (0.0 to 1.0)
            
        Returns:
            Distance in meters
        """
        if height_ratio <= 0:
            return self.fallback_distance
        
        # Use calibration data for interpolation
        distances = sorted(self.calibration_data.keys())
        ratios = [self.calibration_data[d] for d in distances]
        
        # Find interpolation points
        if height_ratio >= max(ratios):
            # Closer than calibration data
            return distances[0]
        elif height_ratio <= min(ratios):
            # Farther than calibration data
            return distances[-1]
        else:
            # Interpolate between calibration points
            for i in range(len(distances) - 1):
                if ratios[i+1] <= height_ratio <= ratios[i]:
                    # Linear interpolation
                    t = (height_ratio - ratios[i+1]) / (ratios[i] - ratios[i+1])
                    distance = distances[i+1] + t * (distances[i] - distances[i+1])
                    return distance
        
        return self.fallback_distance
    
    def _width_ratio_to_distance(self, width_ratio: float) -> float:
        """Convert width ratio to distance.
        
        Args:
            width_ratio: Bbox width / image width (0.0 to 1.0)
            
        Returns:
            Distance in meters
        """
        if width_ratio <= 0:
            return self.fallback_distance
        
        # Use similar approach as height but with width calibration
        # This is less reliable due to person orientation
        width_calibration = {
            1.0: 0.3,   # 1m -> 30% of image width
            2.0: 0.15,  # 2m -> 15% of image width
            3.0: 0.1,   # 3m -> 10% of image width
            4.0: 0.075, # 4m -> 7.5% of image width
            5.0: 0.06,  # 5m -> 6% of image width
            10.0: 0.03, # 10m -> 3% of image width
            20.0: 0.015 # 20m -> 1.5% of image width
        }
        
        distances = sorted(width_calibration.keys())
        ratios = [width_calibration[d] for d in distances]
        
        if width_ratio >= max(ratios):
            return distances[0]
        elif width_ratio <= min(ratios):
            return distances[-1]
        else:
            for i in range(len(distances) - 1):
                if ratios[i+1] <= width_ratio <= ratios[i]:
                    t = (width_ratio - ratios[i+1]) / (ratios[i] - ratios[i+1])
                    distance = distances[i+1] + t * (distances[i] - distances[i+1])
                    return distance
        
        return self.fallback_distance
    
    def calibrate(self, bbox: Tuple[int, int, int, int], actual_distance: float) -> None:
        """Add calibration data point.
        
        Args:
            bbox: Bounding box (x1, y1, x2, y2)
            actual_distance: Actual distance in meters
        """
        x1, y1, x2, y2 = bbox
        bbox_height = y2 - y1
        height_ratio = bbox_height / self.camera_height
        
        # Add to calibration data
        self.calibration_data[actual_distance] = height_ratio
        
        logger.info(f"Added calibration point: {actual_distance}m -> {height_ratio:.3f} height ratio")
    
    def get_calibration_data(self) -> Dict[float, float]:
        """Get current calibration data.
        
        Returns:
            Dictionary of distance -> height_ratio mappings
        """
        return self.calibration_data.copy()
    
    def reset_calibration(self) -> None:
        """Reset to default calibration data."""
        self.calibration_data = {
            1.0: 0.8,   # 1m -> 80% of image height
            2.0: 0.4,   # 2m -> 40% of image height
            3.0: 0.27,  # 3m -> 27% of image height
            4.0: 0.2,   # 4m -> 20% of image height
            5.0: 0.16,  # 5m -> 16% of image height
            10.0: 0.08, # 10m -> 8% of image height
            20.0: 0.04  # 20m -> 4% of image height
        }
        logger.info("Calibration data reset to defaults")
    
    def get_status(self) -> Dict[str, Any]:
        """Get depth estimator status.
        
        Returns:
            Dictionary containing status information
        """
        return {
            'camera_width': self.camera_width,
            'camera_height': self.camera_height,
            'fov_horizontal': self.fov_horizontal,
            'fov_vertical': self.fov_vertical,
            'min_distance': self.min_distance,
            'max_distance': self.max_distance,
            'fallback_distance': self.fallback_distance,
            'calibration_points': len(self.calibration_data)
        }
