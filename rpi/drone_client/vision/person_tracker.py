"""
Simple IoU-based person tracker for persistent ID assignment.
Modular design for easy upgrade to ByteTrack/SORT later.
"""

import numpy as np
import logging
from typing import List, Dict, Any, Tuple, Optional
from collections import defaultdict

logger = logging.getLogger(__name__)

class PersonTracker:
    """Simple IoU-based person tracker."""
    
    def __init__(self, iou_threshold: float = 0.3, max_disappeared: int = 10):
        """Initialize person tracker.
        
        Args:
            iou_threshold: IoU threshold for matching detections
            max_disappeared: Max frames before removing person
        """
        self.iou_threshold = iou_threshold
        self.max_disappeared = max_disappeared
        
        # Tracking state
        self.next_id = 1
        self.tracked_persons = {}  # {id: PersonTrack}
        self.disappeared_count = defaultdict(int)
        
    def update(self, detections: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """Update tracker with new detections.
        
        Args:
            detections: List of person detections from YOLO
            
        Returns:
            List of tracked persons with IDs
        """
        # Filter only chair detections (changed from person for safe testing)
        person_detections = [d for d in detections if d['class_name'] == 'chair']
        
        if len(person_detections) == 0:
            # No detections, increment disappeared count
            for person_id in list(self.tracked_persons.keys()):
                self.disappeared_count[person_id] += 1
                if self.disappeared_count[person_id] > self.max_disappeared:
                    self._remove_person(person_id)
            return []
        
        # Match detections to existing tracks
        matched_ids, unmatched_detections = self._match_detections(person_detections)
        
        # Update matched tracks
        for detection, person_id in matched_ids:
            self._update_person(person_id, detection)
            self.disappeared_count[person_id] = 0
        
        # Create new tracks for unmatched detections
        for detection in unmatched_detections:
            self._create_person(detection)
        
        # Return current tracked persons
        return self._get_tracked_persons()
    
    def _match_detections(self, detections: List[Dict[str, Any]]) -> Tuple[List[Tuple[Dict, int]], List[Dict]]:
        """Match detections to existing tracks.
        
        Args:
            detections: List of person detections
            
        Returns:
            Tuple of (matched_pairs, unmatched_detections)
        """
        if not self.tracked_persons:
            return [], detections
        
        # Calculate IoU matrix
        iou_matrix = self._calculate_iou_matrix(detections)
        
        # Find matches using greedy assignment
        matched_pairs = []
        unmatched_detections = []
        used_track_ids = set()
        
        # Sort by IoU score (highest first)
        matches = []
        for i, detection in enumerate(detections):
            for person_id, person in self.tracked_persons.items():
                if person_id not in used_track_ids:
                    iou = iou_matrix[i][person_id]
                    if iou > self.iou_threshold:
                        matches.append((iou, i, person_id))
        
        matches.sort(reverse=True)  # Sort by IoU score
        
        for iou, det_idx, person_id in matches:
            if person_id not in used_track_ids:
                matched_pairs.append((detections[det_idx], person_id))
                used_track_ids.add(person_id)
        
        # Find unmatched detections
        matched_det_indices = {detections.index(det) for det, _ in matched_pairs}
        for i, detection in enumerate(detections):
            if i not in matched_det_indices:
                unmatched_detections.append(detection)
        
        return matched_pairs, unmatched_detections
    
    def _calculate_iou_matrix(self, detections: List[Dict[str, Any]]) -> Dict[int, Dict[int, float]]:
        """Calculate IoU matrix between detections and tracks.
        
        Args:
            detections: List of person detections
            
        Returns:
            IoU matrix as nested dictionary
        """
        iou_matrix = {}
        
        for i, detection in enumerate(detections):
            iou_matrix[i] = {}
            for person_id, person in self.tracked_persons.items():
                iou = self._calculate_iou(detection['bbox'], person.bbox)
                iou_matrix[i][person_id] = iou
        
        return iou_matrix
    
    def _calculate_iou(self, bbox1: Tuple[int, int, int, int], bbox2: Tuple[int, int, int, int]) -> float:
        """Calculate IoU between two bounding boxes.
        
        Args:
            bbox1: First bounding box (x1, y1, x2, y2)
            bbox2: Second bounding box (x1, y1, x2, y2)
            
        Returns:
            IoU value (0.0 to 1.0)
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
        
        if union <= 0:
            return 0.0
        
        return intersection / union
    
    def _update_person(self, person_id: int, detection: Dict[str, Any]) -> None:
        """Update existing person track.
        
        Args:
            person_id: Person ID
            detection: Detection data
        """
        if person_id in self.tracked_persons:
            person = self.tracked_persons[person_id]
            person.update(detection)
            logger.debug(f"Updated person {person_id}")
    
    def _create_person(self, detection: Dict[str, Any]) -> None:
        """Create new person track.
        
        Args:
            detection: Detection data
        """
        person_id = self.next_id
        self.next_id += 1
        
        person = PersonTrack(person_id, detection)
        self.tracked_persons[person_id] = person
        self.disappeared_count[person_id] = 0
        
        logger.info(f"Created new person track: {person_id}")
    
    def _remove_person(self, person_id: int) -> None:
        """Remove person track.
        
        Args:
            person_id: Person ID to remove
        """
        if person_id in self.tracked_persons:
            del self.tracked_persons[person_id]
            del self.disappeared_count[person_id]
            logger.info(f"Removed person track: {person_id}")
    
    def _get_tracked_persons(self) -> List[Dict[str, Any]]:
        """Get current tracked persons.
        
        Returns:
            List of tracked persons with IDs
        """
        tracked = []
        for person in self.tracked_persons.values():
            tracked.append(person.to_dict())
        return tracked
    
    def get_person_by_id(self, person_id: int) -> Optional[Dict[str, Any]]:
        """Get specific person by ID.
        
        Args:
            person_id: Person ID
            
        Returns:
            Person data if found, None otherwise
        """
        if person_id in self.tracked_persons:
            return self.tracked_persons[person_id].to_dict()
        return None
    
    def get_all_persons(self) -> List[Dict[str, Any]]:
        """Get all tracked persons.
        
        Returns:
            List of all tracked persons
        """
        return self._get_tracked_persons()
    
    def reset(self) -> None:
        """Reset tracker state."""
        self.next_id = 1
        self.tracked_persons.clear()
        self.disappeared_count.clear()
        logger.info("Person tracker reset")
    
    def get_status(self) -> Dict[str, Any]:
        """Get tracker status.
        
        Returns:
            Dictionary containing tracker status
        """
        return {
            'next_id': self.next_id,
            'num_tracked': len(self.tracked_persons),
            'tracked_ids': list(self.tracked_persons.keys()),
            'iou_threshold': self.iou_threshold,
            'max_disappeared': self.max_disappeared
        }

class PersonTrack:
    """Individual person track."""
    
    def __init__(self, person_id: int, detection: Dict[str, Any]):
        """Initialize person track.
        
        Args:
            person_id: Unique person ID
            detection: Initial detection data
        """
        self.person_id = person_id
        self.bbox = detection['bbox']
        self.confidence = detection['confidence']
        self.center = detection['center']
        self.area = detection['area']
        self.class_name = detection['class_name']
        
        # Tracking history
        self.update_count = 1
        self.last_seen = 0
    
    def update(self, detection: Dict[str, Any]) -> None:
        """Update track with new detection.
        
        Args:
            detection: New detection data
        """
        self.bbox = detection['bbox']
        self.confidence = detection['confidence']
        self.center = detection['center']
        self.area = detection['area']
        self.update_count += 1
        self.last_seen = 0
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary.
        
        Returns:
            Dictionary representation
        """
        return {
            'id': self.person_id,
            'bbox': self.bbox,
            'confidence': self.confidence,
            'center': self.center,
            'area': self.area,
            'class_name': self.class_name,
            'update_count': self.update_count,
            'last_seen': self.last_seen
        }
