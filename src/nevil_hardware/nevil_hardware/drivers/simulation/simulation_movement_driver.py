#!/usr/bin/env python3

"""
Simulation Movement Driver for Nevil-picar v2.0

This module implements the MovementInterface for simulation mode,
providing virtual physics and realistic movement behavior without
requiring physical hardware.
"""

import time
import threading
import math
from typing import Optional, Dict, Any

from nevil_hardware.interfaces.movement_interface import MovementInterface, MovementStatus


class SimulationMovementDriver(MovementInterface):
    """
    Simulation movement driver for virtual robot behavior.
    
    This implementation provides:
    - Virtual physics simulation
    - Realistic movement constraints
    - Configurable simulation parameters
    - Thread-safe operation
    """
    
    def __init__(self, logger=None, config: Optional[Dict[str, Any]] = None):
        """Initialize the simulation movement driver."""
        super().__init__()
        self.logger = logger
        self._simulation_mutex = threading.Lock()
        self._initialized = False
        self._simulation_thread = None
        self._running = False
        
        # Simulation configuration
        self.config = config or {}
        self._update_rate = self.config.get('update_rate', 50.0)  # Hz
        self._max_speed = self.config.get('max_speed', 1.0)  # m/s
        self._max_angular_speed = self.config.get('max_angular_speed', 2.0)  # rad/s
        self._acceleration_limit = self.config.get('acceleration_limit', 2.0)  # m/s²
        self._deceleration_limit = self.config.get('deceleration_limit', 3.0)  # m/s²
        self._wheel_base = self.config.get('wheel_base', 0.15)  # meters
        self._wheel_radius = self.config.get('wheel_radius', 0.033)  # meters
        
        # Virtual robot state
        self._virtual_position = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self._virtual_velocity = {'linear': 0.0, 'angular': 0.0}
        self._target_speeds = {'left': 0.0, 'right': 0.0}
        self._current_speeds = {'left': 0.0, 'right': 0.0}
        
        # Update status
        self._status.backend_type = "simulation"
        self._status.hardware_available = True  # Simulation is always "available"
    
    def initialize(self) -> bool:
        """
        Initialize the simulation environment.
        
        Returns:
            bool: True if initialization successful, False otherwise
        """
        try:
            with self._simulation_mutex:
                if self.logger:
                    self.logger.info('Initializing simulation movement driver...')
                
                # Reset simulation state
                self._reset_simulation_state()
                
                # Start simulation thread
                self._running = True
                self._simulation_thread = threading.Thread(
                    target=self._simulation_loop,
                    daemon=True
                )
                self._simulation_thread.start()
                
                self._initialized = True
                self._status.hardware_available = True
                self._status.error_message = None
                
                if self.logger:
                    self.logger.info('Simulation movement driver initialized successfully')
                
                return True
                
        except Exception as e:
            error_msg = f'Failed to initialize simulation driver: {e}'
            if self.logger:
                self.logger.error(error_msg)
            self._status.error_message = error_msg
            return False
    
    def _reset_simulation_state(self) -> None:
        """Reset all simulation state variables."""
        self._virtual_position = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self._virtual_velocity = {'linear': 0.0, 'angular': 0.0}
        self._target_speeds = {'left': 0.0, 'right': 0.0}
        self._current_speeds = {'left': 0.0, 'right': 0.0}
        
        # Reset status
        self._status.left_motor_speed = 0.0
        self._status.right_motor_speed = 0.0
        self._status.steering_angle = 0.0
        self._status.is_moving = False
    
    def _simulation_loop(self) -> None:
        """Main simulation loop running in separate thread."""
        dt = 1.0 / self._update_rate
        
        while self._running:
            try:
                start_time = time.time()
                
                with self._simulation_mutex:
                    self._update_physics(dt)
                
                # Maintain update rate
                elapsed = time.time() - start_time
                sleep_time = max(0, dt - elapsed)
                time.sleep(sleep_time)
                
            except Exception as e:
                if self.logger:
                    self.logger.error(f'Simulation loop error: {e}')
                time.sleep(dt)
    
    def _update_physics(self, dt: float) -> None:
        """Update virtual physics simulation."""
        # Apply acceleration limits to reach target speeds
        for side in ['left', 'right']:
            target = self._target_speeds[side]
            current = self._current_speeds[side]
            
            if abs(target - current) < 0.001:
                self._current_speeds[side] = target
            else:
                # Apply acceleration/deceleration limits
                if target > current:
                    max_change = self._acceleration_limit * dt
                    self._current_speeds[side] = min(target, current + max_change)
                else:
                    max_change = self._deceleration_limit * dt
                    self._current_speeds[side] = max(target, current - max_change)
        
        # Convert wheel speeds to robot velocity
        left_speed = self._current_speeds['left'] * self._max_speed
        right_speed = self._current_speeds['right'] * self._max_speed
        
        # Differential drive kinematics
        linear_velocity = (left_speed + right_speed) / 2.0
        angular_velocity = (right_speed - left_speed) / self._wheel_base
        
        # Update virtual position
        self._virtual_velocity['linear'] = linear_velocity
        self._virtual_velocity['angular'] = angular_velocity
        
        # Integrate position
        theta = self._virtual_position['theta']
        self._virtual_position['x'] += linear_velocity * math.cos(theta) * dt
        self._virtual_position['y'] += linear_velocity * math.sin(theta) * dt
        self._virtual_position['theta'] += angular_velocity * dt
        
        # Normalize angle
        self._virtual_position['theta'] = math.atan2(
            math.sin(self._virtual_position['theta']),
            math.cos(self._virtual_position['theta'])
        )
        
        # Update status
        self._status.is_moving = abs(linear_velocity) > 0.01 or abs(angular_velocity) > 0.01
        self._status.last_command_time = time.time()
    
    def set_motor_speeds(self, left: float, right: float) -> bool:
        """
        Set motor speeds for differential drive.
        
        Args:
            left: Left motor speed (-1.0 to 1.0)
            right: Right motor speed (-1.0 to 1.0)
            
        Returns:
            bool: True if command successful, False otherwise
        """
        if not self._initialized:
            return False
        
        try:
            with self._simulation_mutex:
                # Constrain speeds to valid range
                left_clamped = max(-1.0, min(1.0, left))
                right_clamped = max(-1.0, min(1.0, right))
                
                # Set target speeds
                self._target_speeds['left'] = left_clamped
                self._target_speeds['right'] = right_clamped
                
                # Update status
                self._status.left_motor_speed = left_clamped
                self._status.right_motor_speed = right_clamped
                self._status.last_command_time = time.time()
                
                if self.logger:
                    self.logger.debug(f'Set motor speeds: left={left_clamped:.3f}, right={right_clamped:.3f}')
                
                return True
                
        except Exception as e:
            error_msg = f'Failed to set motor speeds: {e}'
            if self.logger:
                self.logger.error(error_msg)
            self._status.error_message = error_msg
            return False
    
    def set_steering_angle(self, angle: float) -> bool:
        """
        Set steering angle for front wheels.
        
        Note: In simulation, this is converted to differential drive behavior.
        
        Args:
            angle: Steering angle in degrees (-30 to 30)
            
        Returns:
            bool: True if command successful, False otherwise
        """
        if not self._initialized:
            return False
        
        try:
            with self._simulation_mutex:
                # Constrain angle to valid range
                angle_clamped = max(-30.0, min(30.0, angle))
                
                # Convert steering angle to differential drive
                # This is a simplified model for simulation
                if abs(angle_clamped) < 0.1:
                    # Straight driving
                    speed_diff = 0.0
                else:
                    # Calculate speed difference based on steering angle
                    # Larger angles create more differential
                    speed_diff = angle_clamped / 30.0 * 0.5  # Max 50% speed difference
                
                # Apply steering by modifying current target speeds
                base_speed = (self._target_speeds['left'] + self._target_speeds['right']) / 2.0
                
                if angle_clamped > 0:  # Turn right
                    self._target_speeds['left'] = base_speed + abs(speed_diff)
                    self._target_speeds['right'] = base_speed - abs(speed_diff)
                elif angle_clamped < 0:  # Turn left
                    self._target_speeds['left'] = base_speed - abs(speed_diff)
                    self._target_speeds['right'] = base_speed + abs(speed_diff)
                
                # Constrain to valid range
                self._target_speeds['left'] = max(-1.0, min(1.0, self._target_speeds['left']))
                self._target_speeds['right'] = max(-1.0, min(1.0, self._target_speeds['right']))
                
                # Update status
                self._status.steering_angle = angle_clamped
                self._status.last_command_time = time.time()
                
                if self.logger:
                    self.logger.debug(f'Set steering angle: {angle_clamped:.1f}°')
                
                return True
                
        except Exception as e:
            error_msg = f'Failed to set steering angle: {e}'
            if self.logger:
                self.logger.error(error_msg)
            self._status.error_message = error_msg
            return False
    
    def stop(self) -> bool:
        """
        Stop all movement immediately.
        
        Returns:
            bool: True if stop successful, False otherwise
        """
        if not self._initialized:
            return False
        
        try:
            with self._simulation_mutex:
                # Set all speeds to zero
                self._target_speeds['left'] = 0.0
                self._target_speeds['right'] = 0.0
                
                # For immediate stop in simulation, also set current speeds
                self._current_speeds['left'] = 0.0
                self._current_speeds['right'] = 0.0
                
                # Update status
                self._status.left_motor_speed = 0.0
                self._status.right_motor_speed = 0.0
                self._status.is_moving = False
                self._status.last_command_time = time.time()
                
                if self.logger:
                    self.logger.info('Stopped all motors (simulation)')
                
                return True
                
        except Exception as e:
            error_msg = f'Failed to stop motors: {e}'
            if self.logger:
                self.logger.error(error_msg)
            self._status.error_message = error_msg
            return False
    
    def reset(self) -> bool:
        """
        Reset movement system to default state.
        
        Returns:
            bool: True if reset successful, False otherwise
        """
        if not self._initialized:
            return False
        
        try:
            with self._simulation_mutex:
                # Reset simulation state
                self._reset_simulation_state()
                
                if self.logger:
                    self.logger.info('Reset simulation to default state')
                
                return True
                
        except Exception as e:
            error_msg = f'Failed to reset simulation: {e}'
            if self.logger:
                self.logger.error(error_msg)
            self._status.error_message = error_msg
            return False
    
    def get_status(self) -> MovementStatus:
        """
        Get current movement status.
        
        Returns:
            MovementStatus: Current status of movement system
        """
        return self._status
    
    def cleanup(self) -> None:
        """Clean up resources and stop simulation."""
        if self._running:
            self._running = False
            
            if self._simulation_thread and self._simulation_thread.is_alive():
                self._simulation_thread.join(timeout=1.0)
                
            if self.logger:
                self.logger.info('Cleaned up simulation movement driver')
        
        self._initialized = False
    
    def get_virtual_position(self) -> Dict[str, float]:
        """
        Get current virtual robot position (simulation only).
        
        Returns:
            Dict[str, float]: Position with keys 'x', 'y', 'theta'
        """
        with self._simulation_mutex:
            return self._virtual_position.copy()
    
    def get_virtual_velocity(self) -> Dict[str, float]:
        """
        Get current virtual robot velocity (simulation only).
        
        Returns:
            Dict[str, float]: Velocity with keys 'linear', 'angular'
        """
        with self._simulation_mutex:
            return self._virtual_velocity.copy()
    
    def set_virtual_position(self, x: float, y: float, theta: float) -> None:
        """
        Set virtual robot position (simulation only).
        
        Args:
            x: X position in meters
            y: Y position in meters
            theta: Orientation in radians
        """
        with self._simulation_mutex:
            self._virtual_position['x'] = x
            self._virtual_position['y'] = y
            self._virtual_position['theta'] = theta
            
            if self.logger:
                self.logger.debug(f'Set virtual position: x={x:.3f}, y={y:.3f}, θ={theta:.3f}')