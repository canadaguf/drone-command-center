"""
PID controller for smooth drone movement.
Provides PID control for yaw, pitch, roll, and distance.
"""

import time
import logging
from typing import Dict, Any, Optional

logger = logging.getLogger(__name__)

class PIDController:
    """PID controller for drone movement control."""
    
    def __init__(self, kp: float, ki: float, kd: float, 
                 output_min: float = -100.0, output_max: float = 100.0,
                 integral_limit: float = 100.0):
        """Initialize PID controller.
        
        Args:
            kp: Proportional gain
            ki: Integral gain
            kd: Derivative gain
            output_min: Minimum output value
            output_max: Maximum output value
            integral_limit: Integral windup limit
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        self.integral_limit = integral_limit
        
        # PID state
        self._last_error = 0.0
        self._integral = 0.0
        self._last_time = time.time()
        self._last_output = 0.0
        
    def update(self, error: float, dt: Optional[float] = None) -> float:
        """Update PID controller with new error.
        
        Args:
            error: Current error value
            dt: Time delta (if None, calculated automatically)
            
        Returns:
            PID output value
        """
        current_time = time.time()
        
        if dt is None:
            dt = current_time - self._last_time
            if dt <= 0:
                dt = 0.01  # Minimum time step
        
        # Proportional term
        proportional = self.kp * error
        
        # Integral term
        self._integral += error * dt
        # Apply integral windup protection
        self._integral = max(-self.integral_limit, min(self.integral_limit, self._integral))
        integral = self.ki * self._integral
        
        # Derivative term
        if dt > 0:
            derivative = self.kd * (error - self._last_error) / dt
        else:
            derivative = 0.0
        
        # Calculate output
        output = proportional + integral + derivative
        
        # Apply output limits
        output = max(self.output_min, min(self.output_max, output))
        
        # Update state
        self._last_error = error
        self._last_time = current_time
        self._last_output = output
        
        return output
    
    def reset(self) -> None:
        """Reset PID controller state."""
        self._last_error = 0.0
        self._integral = 0.0
        self._last_time = time.time()
        self._last_output = 0.0
        logger.debug("PID controller reset")
    
    def set_gains(self, kp: float, ki: float, kd: float) -> None:
        """Set PID gains.
        
        Args:
            kp: Proportional gain
            ki: Integral gain
            kd: Derivative gain
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        logger.info(f"PID gains updated: Kp={kp}, Ki={ki}, Kd={kd}")

class PIDManager:
    """Manager for multiple PID controllers."""
    
    def __init__(self, config: Dict[str, Any]):
        """Initialize PID manager.
        
        Args:
            config: PID configuration dictionary
        """
        self.controllers = {}
        
        # Create PID controllers
        for axis, gains in config.items():
            self.controllers[axis] = PIDController(
                kp=gains.get('kp', 0.0),
                ki=gains.get('ki', 0.0),
                kd=gains.get('kd', 0.0),
                output_min=-100.0,
                output_max=100.0
            )
            logger.info(f"Created PID controller for {axis}: Kp={gains.get('kp')}, Ki={gains.get('ki')}, Kd={gains.get('kd')}")
    
    def update(self, errors: Dict[str, float], dt: Optional[float] = None) -> Dict[str, float]:
        """Update all PID controllers.
        
        Args:
            errors: Dictionary of axis errors
            dt: Time delta (if None, calculated automatically)
            
        Returns:
            Dictionary of PID outputs
        """
        outputs = {}
        
        for axis, error in errors.items():
            if axis in self.controllers:
                outputs[axis] = self.controllers[axis].update(error, dt)
            else:
                logger.warning(f"No PID controller for axis: {axis}")
                outputs[axis] = 0.0
        
        return outputs
    
    def reset_all(self) -> None:
        """Reset all PID controllers."""
        for controller in self.controllers.values():
            controller.reset()
        logger.info("All PID controllers reset")
    
    def reset_axis(self, axis: str) -> None:
        """Reset specific axis PID controller.
        
        Args:
            axis: Axis name to reset
        """
        if axis in self.controllers:
            self.controllers[axis].reset()
            logger.info(f"Reset PID controller for {axis}")
        else:
            logger.warning(f"No PID controller for axis: {axis}")
    
    def set_gains(self, axis: str, kp: float, ki: float, kd: float) -> None:
        """Set gains for specific axis.
        
        Args:
            axis: Axis name
            kp: Proportional gain
            ki: Integral gain
            kd: Derivative gain
        """
        if axis in self.controllers:
            self.controllers[axis].set_gains(kp, ki, kd)
        else:
            logger.warning(f"No PID controller for axis: {axis}")
    
    def get_status(self) -> Dict[str, Dict[str, float]]:
        """Get status of all PID controllers.
        
        Returns:
            Dictionary containing PID status for each axis
        """
        status = {}
        for axis, controller in self.controllers.items():
            status[axis] = {
                'kp': controller.kp,
                'ki': controller.ki,
                'kd': controller.kd,
                'integral': controller._integral,
                'last_error': controller._last_error,
                'last_output': controller._last_output
            }
        return status
