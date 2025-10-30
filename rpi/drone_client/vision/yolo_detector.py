"""
YOLO11 detector for person detection using ONNX model.
Based on test_snippets/basic_detection_2.py
"""

import cv2
import numpy as np
import logging
from typing import List, Tuple, Dict, Any, Optional

logger = logging.getLogger(__name__)

class YOLODetector:
    """YOLO11 detector for person detection."""
    
    def __init__(self, model_path: str, input_size: int = 320, 
                 confidence_threshold: float = 0.5, nms_threshold: float = 0.4):
        """Initialize YOLO detector.
        
        Args:
            model_path: Path to ONNX model file
            input_size: Input image size (assumes square)
            confidence_threshold: Minimum confidence for detections
            nms_threshold: NMS threshold for duplicate removal
        """
        self.model_path = model_path
        self.input_size = input_size
        self.confidence_threshold = confidence_threshold
        self.nms_threshold = nms_threshold
        
        # Target classes (person=0, car=2)
        self.target_classes = [0, 2]
        self.class_names = {0: "person", 2: "car"}
        
        # Load model
        self.net = None
        self._load_model()
    
    def _load_model(self) -> bool:
        """Load ONNX model.
        
        Returns:
            True if model loaded successfully, False otherwise
        """
        try:
            logger.info(f"Loading YOLO model from {self.model_path}")
            self.net = cv2.dnn.readNetFromONNX(self.model_path)
            self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
            self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
            logger.info("YOLO model loaded successfully")
            return True
        except Exception as e:
            logger.error(f"Failed to load YOLO model: {e}")
            return False
    
    def detect(self, frame: np.ndarray) -> List[Dict[str, Any]]:
        """Detect objects in frame.
        
        Args:
            frame: Input image (BGR format)
            
        Returns:
            List of detection dictionaries
        """
        if self.net is None:
            # Don't spam logs - only log once per session
            if not hasattr(self, '_model_error_logged'):
                logger.warning(f"YOLO model not loaded at {self.model_path} - detection disabled")
                logger.warning("Vision features will not work. Install model to enable.")
                self._model_error_logged = True
            return []
        
        try:
            # Preprocess image
            blob = cv2.dnn.blobFromImage(
                frame,
                scalefactor=1/255.0,
                size=(self.input_size, self.input_size),
                mean=(0, 0, 0),
                swapRB=True,
                crop=False
            )
            
            # Run inference
            self.net.setInput(blob)
            outputs = self.net.forward()
            
            # Post-process results
            detections = self._post_process(outputs, frame.shape[:2])
            
            return detections
            
        except Exception as e:
            logger.error(f"Detection failed: {e}")
            return []
    
    def _post_process(self, outputs: np.ndarray, img_shape: Tuple[int, int]) -> List[Dict[str, Any]]:
        """Post-process YOLO outputs.
        
        Args:
            outputs: Raw YOLO outputs
            img_shape: Original image shape (height, width)
            
        Returns:
            List of processed detections
        """
        outputs = outputs[0]  # Remove batch dimension
        num_candidates = outputs.shape[1]
        
        boxes = []
        scores = []
        class_ids = []
        
        # Calculate scaling factors
        x_factor = img_shape[1] / self.input_size
        y_factor = img_shape[0] / self.input_size
        
        # Process each candidate
        for i in range(num_candidates):
            class_scores = outputs[4:, i]
            max_score = np.max(class_scores)
            
            if max_score >= self.confidence_threshold:
                class_id = np.argmax(class_scores)
                
                if class_id in self.target_classes:
                    # Extract bounding box (center x, center y, width, height)
                    cx, cy, w, h = outputs[0:4, i]
                    
                    # Convert to image coordinates
                    left = int((cx - w / 2) * x_factor)
                    top = int((cy - h / 2) * y_factor)
                    width = int(w * x_factor)
                    height = int(h * y_factor)
                    
                    # Ensure coordinates are within image bounds
                    left = max(0, min(left, img_shape[1] - 1))
                    top = max(0, min(top, img_shape[0] - 1))
                    width = max(1, min(width, img_shape[1] - left))
                    height = max(1, min(height, img_shape[0] - top))
                    
                    boxes.append([left, top, width, height])
                    scores.append(float(max_score))
                    class_ids.append(class_id)
        
        # Apply NMS
        if len(boxes) > 0:
            indices = cv2.dnn.NMSBoxes(
                boxes, scores, self.confidence_threshold, self.nms_threshold
            )
            
            if len(indices) > 0:
                if isinstance(indices, list):
                    indices = np.array(indices)
                
                final_boxes = [boxes[i] for i in indices.flatten()]
                final_scores = [scores[i] for i in indices.flatten()]
                final_classes = [class_ids[i] for i in indices.flatten()]
                
                # Convert to detection format
                detections = []
                for box, score, class_id in zip(final_boxes, final_scores, final_classes):
                    x, y, w, h = box
                    detections.append({
                        'bbox': (x, y, x + w, y + h),  # (x1, y1, x2, y2)
                        'confidence': score,
                        'class_id': class_id,
                        'class_name': self.class_names[class_id],
                        'center': (x + w // 2, y + h // 2),
                        'area': w * h
                    })
                
                return detections
        
        return []
    
    def draw_detections(self, frame: np.ndarray, detections: List[Dict[str, Any]]) -> np.ndarray:
        """Draw detections on frame.
        
        Args:
            frame: Input frame
            detections: List of detections
            
        Returns:
            Frame with drawn detections
        """
        annotated_frame = frame.copy()
        
        for detection in detections:
            x1, y1, x2, y2 = detection['bbox']
            confidence = detection['confidence']
            class_name = detection['class_name']
            
            # Draw bounding box
            cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
            # Draw label
            label = f"{class_name} {confidence:.2f}"
            label_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)[0]
            cv2.rectangle(annotated_frame, (x1, y1 - label_size[1] - 10), 
                         (x1 + label_size[0], y1), (0, 255, 0), -1)
            cv2.putText(annotated_frame, label, (x1, y1 - 5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
        
        return annotated_frame
    
    def get_model_info(self) -> Dict[str, Any]:
        """Get model information.
        
        Returns:
            Dictionary containing model info
        """
        return {
            'model_path': self.model_path,
            'input_size': self.input_size,
            'confidence_threshold': self.confidence_threshold,
            'nms_threshold': self.nms_threshold,
            'target_classes': self.target_classes,
            'class_names': self.class_names,
            'loaded': self.net is not None
        }
