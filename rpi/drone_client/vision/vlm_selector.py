"""
Optional VLM module for text-based target selection.
Uses Gemini or OpenAI models for natural language person selection.
"""

import logging
import base64
import cv2
import numpy as np
from typing import Dict, Any, Optional, List
import asyncio
import aiohttp
import json

logger = logging.getLogger(__name__)

class VLMTargetSelector:
    """VLM-based target selector for natural language person selection."""
    
    def __init__(self, config: Dict[str, Any]):
        """Initialize VLM selector.
        
        Args:
            config: VLM configuration
        """
        self.enabled = config.get('enabled', False)
        self.provider = config.get('provider', 'gemini')
        self.model = config.get('model', 'gemini-2.0-flash')
        self.api_key = config.get('api_key', '')
        
        if self.enabled and not self.api_key:
            logger.warning("VLM enabled but no API key provided")
            self.enabled = False
        
        self.client = None
        if self.enabled:
            self._initialize_client()
        
        logger.info(f"VLM selector initialized: enabled={self.enabled}, provider={self.provider}")
    
    def _initialize_client(self) -> None:
        """Initialize VLM client."""
        try:
            if self.provider == 'gemini':
                self.client = GeminiClient(self.api_key, self.model)
            elif self.provider == 'openai':
                self.client = OpenAIClient(self.api_key, self.model)
            else:
                logger.error(f"Unsupported VLM provider: {self.provider}")
                self.enabled = False
        except Exception as e:
            logger.error(f"Failed to initialize VLM client: {e}")
            self.enabled = False
    
    async def select_by_description(self, frame: np.ndarray, detections: List[Dict[str, Any]], 
                                  description: str) -> Optional[int]:
        """Select person by natural language description.
        
        Args:
            frame: Camera frame
            detections: List of person detections
            description: Natural language description
            
        Returns:
            Person ID if found, None otherwise
        """
        if not self.enabled or not self.client:
            return None
        
        try:
            # Create annotated frame with bounding boxes
            annotated_frame = self._draw_detections(frame, detections)
            
            # Convert frame to base64
            frame_b64 = self._frame_to_base64(annotated_frame)
            
            # Create prompt
            prompt = self._create_prompt(detections, description)
            
            # Call VLM
            response = await self.client.analyze(frame_b64, prompt)
            
            # Parse response
            person_id = self._parse_response(response)
            
            if person_id is not None:
                logger.info(f"VLM selected person {person_id} for description: '{description}'")
            else:
                logger.warning(f"VLM could not match description: '{description}'")
            
            return person_id
            
        except Exception as e:
            logger.error(f"VLM selection failed: {e}")
            return None
    
    def _draw_detections(self, frame: np.ndarray, detections: List[Dict[str, Any]]) -> np.ndarray:
        """Draw detections on frame.
        
        Args:
            frame: Input frame
            detections: List of detections
            
        Returns:
            Annotated frame
        """
        annotated = frame.copy()
        
        for detection in detections:
            bbox = detection.get('bbox')
            person_id = detection.get('id')
            confidence = detection.get('confidence', 0.0)
            
            if bbox and person_id is not None:
                x1, y1, x2, y2 = bbox
                
                # Draw bounding box
                cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
                
                # Draw ID label
                label = f"ID {person_id}"
                cv2.putText(annotated, label, (x1, y1 - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        return annotated
    
    def _frame_to_base64(self, frame: np.ndarray) -> str:
        """Convert frame to base64 string.
        
        Args:
            frame: Input frame
            
        Returns:
            Base64 encoded frame
        """
        _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        return base64.b64encode(buffer).decode()
    
    def _create_prompt(self, detections: List[Dict[str, Any]], description: str) -> str:
        """Create VLM prompt.
        
        Args:
            detections: List of detections
            description: User description
            
        Returns:
            VLM prompt
        """
        prompt = f"""
        In this image, there are {len(detections)} people detected.
        Each person is marked with a bounding box and ID number.
        
        User wants to follow: "{description}"
        
        Which person ID matches this description? 
        Respond with only the ID number (e.g., "1" or "2").
        If no person matches the description, respond with "None".
        """
        return prompt
    
    def _parse_response(self, response: str) -> Optional[int]:
        """Parse VLM response.
        
        Args:
            response: VLM response
            
        Returns:
            Person ID if found, None otherwise
        """
        try:
            # Extract number from response
            import re
            numbers = re.findall(r'\d+', response)
            if numbers:
                return int(numbers[0])
            return None
        except:
            return None
    
    def is_enabled(self) -> bool:
        """Check if VLM is enabled.
        
        Returns:
            True if enabled, False otherwise
        """
        return self.enabled
    
    def get_status(self) -> Dict[str, Any]:
        """Get VLM status.
        
        Returns:
            Dictionary containing status
        """
        return {
            'enabled': self.enabled,
            'provider': self.provider,
            'model': self.model,
            'has_api_key': bool(self.api_key),
            'client_initialized': self.client is not None
        }

class GeminiClient:
    """Gemini API client."""
    
    def __init__(self, api_key: str, model: str):
        """Initialize Gemini client."""
        self.api_key = api_key
        self.model = model
        self.base_url = "https://generativelanguage.googleapis.com/v1beta"
    
    async def analyze(self, image_b64: str, prompt: str) -> str:
        """Analyze image with Gemini.
        
        Args:
            image_b64: Base64 encoded image
            prompt: Text prompt
            
        Returns:
            Response text
        """
        async with aiohttp.ClientSession() as session:
            url = f"{self.base_url}/models/{self.model}:generateContent"
            
            payload = {
                "contents": [{
                    "parts": [
                        {"text": prompt},
                        {
                            "inline_data": {
                                "mime_type": "image/jpeg",
                                "data": image_b64
                            }
                        }
                    ]
                }]
            }
            
            headers = {
                "Content-Type": "application/json"
            }
            
            params = {"key": self.api_key}
            
            async with session.post(url, json=payload, headers=headers, params=params) as response:
                if response.status == 200:
                    data = await response.json()
                    return data["candidates"][0]["content"]["parts"][0]["text"]
                else:
                    raise Exception(f"Gemini API error: {response.status}")

class OpenAIClient:
    """OpenAI API client."""
    
    def __init__(self, api_key: str, model: str):
        """Initialize OpenAI client."""
        self.api_key = api_key
        self.model = model
        self.base_url = "https://api.openai.com/v1"
    
    async def analyze(self, image_b64: str, prompt: str) -> str:
        """Analyze image with OpenAI.
        
        Args:
            image_b64: Base64 encoded image
            prompt: Text prompt
            
        Returns:
            Response text
        """
        async with aiohttp.ClientSession() as session:
            url = f"{self.base_url}/chat/completions"
            
            payload = {
                "model": self.model,
                "messages": [
                    {
                        "role": "user",
                        "content": [
                            {"type": "text", "text": prompt},
                            {
                                "type": "image_url",
                                "image_url": {
                                    "url": f"data:image/jpeg;base64,{image_b64}"
                                }
                            }
                        ]
                    }
                ],
                "max_tokens": 100
            }
            
            headers = {
                "Authorization": f"Bearer {self.api_key}",
                "Content-Type": "application/json"
            }
            
            async with session.post(url, json=payload, headers=headers) as response:
                if response.status == 200:
                    data = await response.json()
                    return data["choices"][0]["message"]["content"]
                else:
                    raise Exception(f"OpenAI API error: {response.status}")
