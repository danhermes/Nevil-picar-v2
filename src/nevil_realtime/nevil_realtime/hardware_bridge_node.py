#!/usr/bin/env python3

"""
Hardware Bridge Node for Nevil-picar v2.0.

This node provides a ROS2 service interface to the hardware abstraction layer,
enabling C++ motion control nodes to execute actual hardware commands through
the Python hardware interface with v1.0 action helper integration.
"""

import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
import threading
from typing import Dict, Any

from nevil_interfaces.srv import HardwareCommand
from .rt_hardware_interface import RTHardwareInterface


class HardwareBridgeNode(Node):
    """
    Hardware Bridge Node that provides ROS2 service interface to hardware.
    
    This node acts as a bridge between C++ motion control nodes and the
    Python hardware abstraction layer, enabling actual physical movement
    execution through the v1.0 action helper integration.
    """
    
    def __init__(self):
        super().__init__('hardware_bridge_node')
        
        # Declare parameters
        self.declare_parameter('hardware_backend', 'auto')
        self.declare_parameter('use_sim', False)
        self.declare_parameter('config_file', '')
        
        # Get parameters
        backend = self.get_parameter('hardware_backend').get_parameter_value().string_value
        use_sim = self.get_parameter('use_sim').get_parameter_value().bool_value
        config_file = self.get_parameter('config_file').get_parameter_value().string_value
        
        # Override backend based on use_sim parameter
        if use_sim:
            backend = 'simulation'
        elif backend == 'auto' and not use_sim:
            backend = 'auto'  # Will try physical first, fallback to simulation
        
        self.get_logger().info(f'Hardware Bridge Node starting with backend: {backend}')
        
        # Initialize hardware interface
        config = {}
        if config_file:
            config['config_file'] = config_file
        
        try:
            self.hardware = RTHardwareInterface(
                node=self,
                backend=backend,
                config=config
            )
            self.get_logger().info(f'Hardware interface initialized: {self.hardware.get_backend_type()}')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize hardware interface: {e}')
            # Emergency fallback
            self.hardware = RTHardwareInterface(node=self, backend='simulation')
            self.get_logger().warn('Using emergency simulation fallback')
        
        # Create service server
        self.service = self.create_service(
            HardwareCommand,
            'hardware_command',
            self.hardware_command_callback
        )
        
        # Thread safety
        self.command_mutex = threading.Lock()
        
        self.get_logger().info('Hardware Bridge Node ready to receive commands')
    
    def hardware_command_callback(self, request, response):
        """
        Handle hardware command service requests.
        
        Args:
            request: HardwareCommand request with movement parameters
            response: HardwareCommand response with execution status
            
        Returns:
            HardwareCommand.Response: Service response with execution results
        """
        with self.command_mutex:
            try:
                self.get_logger().debug(
                    f'Received hardware command: linear={request.linear_x:.2f}, '
                    f'angular={request.angular_z:.2f}, emergency={request.emergency_stop}, '
                    f'type={request.command_type}'
                )
                
                # Handle emergency stop
                if request.emergency_stop:
                    self.hardware.stop()
                    response.success = True
                    response.status_message = "Emergency stop executed"
                    response.hardware_backend = self.hardware.get_backend_type()
                    response.actual_linear_x = 0.0
                    response.actual_angular_z = 0.0
                    response.hardware_ready = True
                    return response
                
                # Execute movement command based on type
                if request.command_type == "velocity":
                    result = self._execute_velocity_command(request.linear_x, request.angular_z)
                elif request.command_type == "position":
                    result = self._execute_position_command(request.linear_x, request.angular_z)
                else:
                    result = self._execute_velocity_command(request.linear_x, request.angular_z)
                
                # Fill response
                response.success = result.get('success', True)
                response.status_message = result.get('message', 'Command executed')
                response.hardware_backend = self.hardware.get_backend_type()
                response.actual_linear_x = result.get('actual_linear_x', request.linear_x)
                response.actual_angular_z = result.get('actual_angular_z', request.angular_z)
                response.hardware_ready = not self.hardware.simulation_mode
                
                if response.success:
                    self.get_logger().debug(f'Hardware command executed successfully: {response.status_message}')
                else:
                    self.get_logger().warn(f'Hardware command failed: {response.status_message}')
                
                return response
                
            except Exception as e:
                self.get_logger().error(f'Hardware command execution failed: {e}')
                response.success = False
                response.status_message = f"Hardware command failed: {str(e)}"
                response.hardware_backend = self.hardware.get_backend_type()
                response.actual_linear_x = 0.0
                response.actual_angular_z = 0.0
                response.hardware_ready = False
                return response
    
    def _execute_velocity_command(self, linear_x: float, angular_z: float) -> Dict[str, Any]:
        """
        Execute velocity-based movement command.
        
        Args:
            linear_x: Linear velocity in m/s
            angular_z: Angular velocity in rad/s
            
        Returns:
            Dict with execution results
        """
        try:
            # Convert ROS velocity to motor commands
            # This is a simplified differential drive model
            wheel_separation = 0.14  # meters (approximate for PiCar-X)
            wheel_radius = 0.03      # meters (approximate for PiCar-X)
            
            # Calculate left and right wheel speeds
            left_speed = (linear_x - angular_z * wheel_separation / 2.0) / wheel_radius
            right_speed = (linear_x + angular_z * wheel_separation / 2.0) / wheel_radius
            
            # Normalize to -1.0 to 1.0 range
            max_speed = 2.0  # rad/s (approximate max wheel speed)
            left_normalized = max(-1.0, min(1.0, left_speed / max_speed))
            right_normalized = max(-1.0, min(1.0, right_speed / max_speed))
            
            # Execute motor command
            self.hardware.set_motor_speeds(left_normalized, right_normalized)
            
            # Handle steering for car-like motion (if angular velocity is significant)
            if abs(angular_z) > 0.1:  # threshold for steering
                steering_angle = max(-30, min(30, angular_z * 30))  # Convert to degrees
                self.hardware.set_steering_angle(steering_angle)
            else:
                self.hardware.set_steering_angle(0)
            
            return {
                'success': True,
                'message': f'Velocity command executed: linear={linear_x:.2f}, angular={angular_z:.2f}',
                'actual_linear_x': linear_x,
                'actual_angular_z': angular_z
            }
            
        except Exception as e:
            return {
                'success': False,
                'message': f'Velocity command failed: {str(e)}',
                'actual_linear_x': 0.0,
                'actual_angular_z': 0.0
            }
    
    def _execute_position_command(self, distance: float, angle: float) -> Dict[str, Any]:
        """
        Execute position-based movement command.
        
        Args:
            distance: Distance to move in meters
            angle: Angle to turn in radians
            
        Returns:
            Dict with execution results
        """
        try:
            # This would implement position-based movement
            # For now, convert to velocity command
            linear_velocity = 0.2 if distance > 0 else -0.2 if distance < 0 else 0.0
            angular_velocity = 0.5 if angle > 0 else -0.5 if angle < 0 else 0.0
            
            return self._execute_velocity_command(linear_velocity, angular_velocity)
            
        except Exception as e:
            return {
                'success': False,
                'message': f'Position command failed: {str(e)}',
                'actual_linear_x': 0.0,
                'actual_angular_z': 0.0
            }
    
    def destroy_node(self):
        """Clean up hardware resources when node is destroyed."""
        try:
            if hasattr(self, 'hardware'):
                self.hardware.cleanup()
        except Exception as e:
            self.get_logger().error(f'Error during hardware cleanup: {e}')
        finally:
            super().destroy_node()


def main(args=None):
    """Main function for the hardware bridge node."""
    rclpy.init(args=args)
    
    try:
        node = HardwareBridgeNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Hardware bridge node failed: {e}')
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()