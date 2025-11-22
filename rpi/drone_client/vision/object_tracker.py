"""
Object tracker for assigning and maintaining IDs across frames.
Supports DeepSORT-realtime with fallback to simple IoU-based tracker.
"""

import logging
import numpy as np
from typing import List, Dict, Any, Optional, Tuple
import cv2

logger = logging.getLogger(__name__)

# Try to import deep-sort-realtime, fallback to simple tracker
try:
    from deep_sort_realtime import DeepSort
    DEEPSORT_AVAILABLE = True
except ImportError:
    DEEPSORT_AVAILABLE = False
    logger.warning("deep-sort-realtime not available, using simple tracker")


class SimpleTracker:
    """Simple IoU-based object tracker."""
    
    def __init__(self, max_age: int = 30, min_hits: int = 3, iou_threshold: float = 0.3):
        """Initialize simple tracker.
        
        Args:
            max_age: Maximum frames to keep track without detection
            min_hits: Minimum detections before confirming track
            iou_threshold: IoU threshold for matching detections to tracks
        """
        self.max_age = max_age
        self.min_hits = min_hits
        self.iou_threshold = iou_threshold
        
        self.tracks = {}  # track_id -> track_data
        self.next_id = 1
        self.frame_count = 0
    
    def update(self, detections: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """Update tracker with new detections.
        
        Args:
            detections: List of detections with 'bbox' (x1, y1, x2, y2)
            
        Returns:
            List of tracked detections with 'id' field added
        """
        self.frame_count += 1
        
        if len(detections) == 0:
            # No detections - age all tracks
            for track_id in list(self.tracks.keys()):
                self.tracks[track_id]['age'] += 1
                if self.tracks[track_id]['age'] > self.max_age:
                    del self.tracks[track_id]
            return []
        
        # Calculate IoU matrix
        track_ids = list(self.tracks.keys())
        iou_matrix = np.zeros((len(track_ids), len(detections)))
        
        for i, track_id in enumerate(track_ids):
            track_bbox = self.tracks[track_id]['bbox']
            for j, det in enumerate(detections):
                iou_matrix[i, j] = self._calculate_iou(track_bbox, det['bbox'])
        
        # Match detections to tracks (greedy matching)
        matched_tracks = set()
        matched_detections = set()
        tracked_detections = []
        
        # Sort by IoU (highest first)
        matches = []
        for i, track_id in enumerate(track_ids):
            for j in range(len(detections)):
                if iou_matrix[i, j] > self.iou_threshold:
                    matches.append((iou_matrix[i, j], i, j))
        
        matches.sort(reverse=True)
        
        for iou, i, j in matches:
            if i not in matched_tracks and j not in matched_detections:
                track_id = track_ids[i]
                # Update track
                self.tracks[track_id]['bbox'] = detections[j]['bbox']
                self.tracks[track_id]['age'] = 0
                self.tracks[track_id]['hits'] += 1
                
                # Add ID to detection
                tracked_det = detections[j].copy()
                tracked_det['id'] = track_id
                tracked_detections.append(tracked_det)
                
                matched_tracks.add(i)
                matched_detections.add(j)
        
        # Create new tracks for unmatched detections
        for j, det in enumerate(detections):
            if j not in matched_detections:
                track_id = self.next_id
                self.next_id += 1
                
                self.tracks[track_id] = {
                    'bbox': det['bbox'],
                    'age': 0,
                    'hits': 1
                }
                
                tracked_det = det.copy()
                tracked_det['id'] = track_id
                tracked_detections.append(tracked_det)
        
        # Age unmatched tracks
        for track_id in track_ids:
            if track_id not in matched_tracks:
                self.tracks[track_id]['age'] += 1
                if self.tracks[track_id]['age'] > self.max_age:
                    del self.tracks[track_id]
        
        # Only return confirmed tracks (min_hits)
        confirmed_detections = [
            det for det in tracked_detections
            if self.tracks[det['id']]['hits'] >= self.min_hits
        ]
        
        return confirmed_detections
    
    def _calculate_iou(self, bbox1: Tuple[int, int, int, int], 
                      bbox2: Tuple[int, int, int, int]) -> float:
        """Calculate Intersection over Union (IoU) of two bounding boxes.
        
        Args:
            bbox1: (x1, y1, x2, y2)
            bbox2: (x1, y1, x2, y2)
            
        Returns:
            IoU value between 0 and 1
        """
        x1_1, y1_1, x2_1, y2_1 = bbox1
        x1_2, y1_2, x2_2, y2_2 = bbox2
        
        # Calculate intersection
        x1_i = max(x1_1, x1_2)
        y1_i = max(y1_1, y1_2)
        x2_i = min(x2_1, x2_2)
        y2_i = min(y2_1, y2_2)
        
        if x2_i <= x1_i or y2_i <= y1_i:
            return 0.0
        
        intersection = (x2_i - x1_i) * (y2_i - y1_i)
        
        # Calculate union
        area1 = (x2_1 - x1_1) * (y2_1 - y1_1)
        area2 = (x2_2 - x1_2) * (y2_2 - y1_2)
        union = area1 + area2 - intersection
        
        if union == 0:
            return 0.0
        
        return intersection / union


class ObjectTracker:
    """Object tracker with DeepSORT support and simple tracker fallback."""
    
    def __init__(self, use_deepsort: bool = True):
        """Initialize object tracker.
        
        Args:
            use_deepsort: If True, try to use DeepSORT (falls back to simple if unavailable)
        """
        self.use_deepsort = use_deepsort and DEEPSORT_AVAILABLE
        self.tracker = None
        
        if self.use_deepsort:
            try:
                # Initialize DeepSORT tracker
                self.tracker = DeepSort(
                    max_age=30,
                    n_init=3,
                    max_iou_distance=0.7,
                    max_cosine_distance=0.2,
                    nn_budget=100
                )
                logger.info("Using DeepSORT-realtime tracker")
            except Exception as e:
                logger.warning(f"Failed to initialize DeepSORT: {e}, falling back to simple tracker")
                self.use_deepsort = False
        
        if not self.use_deepsort:
            self.tracker = SimpleTracker(max_age=30, min_hits=3, iou_threshold=0.3)
            logger.info("Using simple IoU-based tracker")
    
    def update(self, detections: List[Dict[str, Any]], 
              frame: Optional[np.ndarray] = None) -> List[Dict[str, Any]]:
        """Update tracker with new detections.
        
        Args:
            detections: List of detections with 'bbox' (x1, y1, x2, y2), 'confidence', 'class_id'
            frame: Optional frame for DeepSORT feature extraction
            
        Returns:
            List of tracked detections with 'id' field added
        """
        if len(detections) == 0:
            if self.use_deepsort:
                # DeepSORT needs to be updated even with no detections
                tracks = self.tracker.update_tracks([], frame=frame)
                return []
            else:
                return self.tracker.update(detections)
        
        if self.use_deepsort:
            # Convert detections to DeepSORT format
            detections_list = []
            for det in detections:
                x1, y1, x2, y2 = det['bbox']
                conf = det.get('confidence', 0.5)
                cls = det.get('class_id', 0)
                
                # DeepSORT expects [x1, y1, w, h]
                detections_list.append(([x1, y1, x2 - x1, y2 - y1], conf, cls))
            
            # Update DeepSORT tracker
            tracks = self.tracker.update_tracks(detections_list, frame=frame)
            
            # Convert back to our format
            tracked_detections = []
            for track in tracks:
                if not track.is_confirmed():
                    continue
                
                # Get track ID
                track_id = track.track_id
                
                # Get bounding box (ltrb format)
                ltrb = track.to_ltrb()
                x1, y1, x2, y2 = int(ltrb[0]), int(ltrb[1]), int(ltrb[2]), int(ltrb[3])
                
                # Find matching detection (by bbox overlap)
                best_match = None
                best_iou = 0
                for det in detections:
                    iou = self._calculate_iou((x1, y1, x2, y2), det['bbox'])
                    if iou > best_iou:
                        best_iou = iou
                        best_match = det
                
                if best_match:
                    tracked_det = best_match.copy()
                    tracked_det['id'] = track_id
                    tracked_det['bbox'] = (x1, y1, x2, y2)  # Use tracked bbox
                    tracked_detections.append(tracked_det)
            
            return tracked_detections
        else:
            # Use simple tracker
            return self.tracker.update(detections)
    
    def _calculate_iou(self, bbox1: Tuple[int, int, int, int], 
                      bbox2: Tuple[int, int, int, int]) -> float:
        """Calculate IoU between two bounding boxes."""
        x1_1, y1_1, x2_1, y2_1 = bbox1
        x1_2, y1_2, x2_2, y2_2 = bbox2
        
        x1_i = max(x1_1, x1_2)
        y1_i = max(y1_1, y1_2)
        x2_i = min(x2_1, x2_2)
        y2_i = min(y2_1, y2_2)
        
        if x2_i <= x1_i or y2_i <= y1_i:
            return 0.0
        
        intersection = (x2_i - x1_i) * (y2_i - y1_i)
        area1 = (x2_1 - x1_1) * (y2_1 - y1_1)
        area2 = (x2_2 - x1_2) * (y2_2 - y1_2)
        union = area1 + area2 - intersection
        
        return intersection / union if union > 0 else 0.0

