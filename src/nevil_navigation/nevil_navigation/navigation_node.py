#!/usr/bin/env python3

import sys
import os
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import json
import math
from .picar_actions import PicarActions
#from robot_hat import Pin, ADC, PWM, Servo, utils

# # Fix the robot_hat.utils.reset_mcu issue before importing Picarx
# try:
#     #import robot_hat.utils
#     #from robot_hat import reset_mcu
#     # Monkey patch the missing reset_mcu function with timeout protection
#     if not hasattr(robot_hat.utils, 'reset_mcu'):
#         def safe_reset_mcu():
#             import signal
#             import time
#             def timeout_handler(signum, frame):
#                 raise TimeoutError("reset_mcu() timed out")
            
#             signal.signal(signal.SIGALRM, timeout_handler)
#             signal.alarm(3)  # 3 second timeout
#             try:
#                 reset_mcu()
#                 signal.alarm(0)  # Cancel timeout
#                 time.sleep(0.1)
#             except TimeoutError:
#                 signal.alarm(0)  # Cancel timeout
#                 print("reset_mcu() timed out - continuing anyway")
#             except Exception as e:
#                 signal.alarm(0)  # Cancel timeout
#                 print(f"reset_mcu() failed: {e}")
        
#         robot_hat.utils.reset_mcu = safe_reset_mcu
# except ImportError:
#     pass

from .picarx import Picarx

# Import AI command message
try:
    from nevil_interfaces_ai_msgs.msg import AICommand
except ImportError:
    # Fallback if AI messages not available
    AICommand = None


class NavigationNode(Node):
    """
    Navigation Node for Nevil-picar v2.0
    
    This node is responsible for:
    - Path planning and execution
    - Integrating sensor data for navigation
    - Implementing navigation behaviors
    - Coordinating with obstacle avoidance
    """
    
    def __init__(self):
        super().__init__('navigation_node')
        
        # Validate ROS2 context before proceeding
        if not self._validate_ros_context():
            raise RuntimeError("ROS2 context is invalid - cannot initialize NavigationNode")
        
        # Initialize PiCar hardware using v1.0 approach
        try:
            self.get_logger().info("Using ROS2 communication with rt_motor_control_node (no direct hardware)")
            
            # Don't initialize hardware directly - communicate via ROS topics
            # The rt_motor_control_node handles all hardware interactions
            self.car = None  # No direct hardware access
            
            # Set default parameters without hardware initialization
            self.speed = 30  # Set default speed
            self.DEFAULT_HEAD_TILT = 20

            self.get_logger().info("Navigation node configured for ROS2 communication")

            # Initialize PicarAction without hardware (simulation mode actions)
            self.picar_actions = PicarActions(car_instance=None)

            self.get_logger().info("PicarActions initialized for ROS2 communication.")


        except Exception as e:
            self.car = None
            self.get_logger().warn(f"Failed to initialize PiCar hardware: {e}")
            self.get_logger().info("Running in simulation mode without hardware")

        # QoS profile for navigation messages
        nav_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Initialize parameters with context validation
        self._safe_declare_parameters()
        
        # Create publishers
        self.cmd_vel_publisher = self.create_publisher(
            Twist, 
            '/cmd_vel', 
            qos_profile=nav_qos
        )
        
        self.path_publisher = self.create_publisher(
            Path,
            '/planned_path',
            qos_profile=nav_qos
        )
        
        # Create subscribers
        self.goal_subscription = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )
        
        self.system_mode_subscription = self.create_subscription(
            String,
            '/system_mode',
            self.system_mode_callback,
            10
        )
        
        # Subscribe to AI action commands if available

        if AICommand is not None:
            self.action_command_subscription = self.create_subscription(
                AICommand,
                '/nevil/action_command',
                self.action_command_callback,
                qos_profile=nav_qos
            )
            self.get_logger().info('Subscribed to AI action commands')
        else:
            self.get_logger().warning('AI command messages not available, action execution disabled')
        
        # Create timers
        self.timer = self.create_timer(0.1, self.navigation_loop)
        
        # State variables
        self.current_goal = None
        self.current_path = None
        self.system_mode = 'active'  # Start in active mode, will be updated by system manager
        self.navigation_active = False

        self.get_logger().info('Navigation Node initialized')
    
    def _validate_ros_context(self):
        """Validate that ROS2 context is still valid"""
        try:
            return rclpy.ok() and self.context.ok()
        except Exception:
            return False
    
    def _safe_declare_parameters(self):
        """Safely declare parameters with context validation"""
        try:
            if not self._validate_ros_context():
                self.get_logger().error("Cannot declare parameters: ROS2 context is invalid")
                # Use default values instead of crashing
                self._navigation_mode = 'manual'
                self._max_speed = 0.5
                return
            
            # Declare parameters with error handling
            self.declare_parameter('navigation_mode', 'manual')  # manual, autonomous, learning
            self.declare_parameter('max_speed', 0.5)  # m/s
            
        except Exception as e:
            self.get_logger().error(f"Failed to declare parameters: {e}")
            # Use default values as fallback
            self._navigation_mode = 'manual'
            self._max_speed = 0.5
    
    def _safe_get_parameter(self, param_name, default_value):
        """Safely get parameter value with fallback"""
        try:
            if not self._validate_ros_context():
                return default_value
            return self.get_parameter(param_name).get_parameter_value()
        except Exception:
            return default_value
        
    def goal_callback(self, msg):
        """Process new navigation goal"""
        self.current_goal = msg
        self.get_logger().info(f'Received new goal: x={msg.pose.position.x}, y={msg.pose.position.y}')
        
        # Here we would implement path planning
        # For now, just set navigation active
        if self.system_mode != 'standby':
            self.navigation_active = True
            self.get_logger().info('Starting navigation to goal')
        else:
            self.get_logger().warn('Cannot navigate while in standby mode')
    
    def system_mode_callback(self, msg):
        """Handle system mode changes"""
        self.system_mode = msg.data
        
        if self.system_mode == 'standby':
            self.navigation_active = False
            self.stop_robot()
            # Reset servos to straight position when entering standby mode
            if hasattr(self, 'picar_actions') and self.picar_actions is not None:
                try:
                    self.picar_actions.initialize_servos()
                    self.get_logger().info('Servos reset to straight position for standby mode')
                except Exception as e:
                    self.get_logger().error(f'Failed to reset servos for standby mode: {e}')
            self.get_logger().info('Navigation stopped due to standby mode')
    
    def navigation_loop(self):
        """Main navigation control loop"""
        if not self.navigation_active or self.current_goal is None:
            return
            
        # Here we would implement path following
        # For now, just send a simple command
        cmd = Twist()
        
        # Simple example: move forward at half speed
        try:
            nav_mode_param = self._safe_get_parameter('navigation_mode', 'manual')
            max_speed_param = self._safe_get_parameter('max_speed', 0.5)
            
            nav_mode = nav_mode_param.string_value if hasattr(nav_mode_param, 'string_value') else nav_mode_param
            max_speed = max_speed_param.double_value if hasattr(max_speed_param, 'double_value') else max_speed_param
        except Exception as e:
            # Fallback values if parameter access fails
            nav_mode = 'manual'
            max_speed = 0.5
            self.get_logger().debug(f'Parameter access failed, using defaults: {e}')
        
        if nav_mode == 'autonomous' and self.navigation_active:
            cmd.linear.x = max_speed / 2.0
            self.cmd_vel_publisher.publish(cmd)
            self.get_logger().debug(f'Sending cmd_vel: linear.x={cmd.linear.x}')
    
    def action_command_callback(self, msg):
        """Handle AI action commands"""
        self.get_logger().info(f'Received action command: {msg.command_type}')
        
        try:
            # Parse command data
            command_data = json.loads(msg.command_data) if msg.command_data else {}

            self.get_logger().info(f'Action executed in Nav node: {msg.command_type}')

            # Execute the action based on command type
            self.execute_action(msg.command_type, command_data)
            
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid JSON in action command data: {e}')
        except Exception as e:
            self.get_logger().error(f'Error processing action command: {e}')
    
    def execute_action(self, action_type, action_data):
        """Execute a specific action using ROS2 communication to rt_motor_control_node"""
        self.get_logger().info(f'Executing action via ROS2: {action_type}.')
        
        # Send movement commands via ROS topics to rt_motor_control_node
        # This bypasses direct hardware access and uses the hardware abstraction layer
        
        # Handle movement actions via ROS2 cmd_vel publishing to rt_motor_control_node
        if action_type in ["forward", "backward", "left", "right", "stop"]:
            self._execute_movement_action(action_type, action_data)
        
        # Handle gesture/behavior actions (placeholder for now)
        elif action_type in ["shake_head", "nod", "wave_hands", "resist", "act_cute", "rub_hands", 
                           "think", "twist_body", "celebrate", "depressed", "keep_think", "honk", "start_engine"]:
            self._execute_gesture_action(action_type, action_data)
        
        else:
            self.get_logger().warning(f'Unknown action type: {action_type}')

    def _execute_movement_action(self, action_type, action_data):
        """Execute movement actions by publishing cmd_vel messages to rt_motor_control_node"""
        from geometry_msgs.msg import Twist
        
        cmd = Twist()
        duration = action_data.get('duration', 2.0)  # Default 2 seconds
        
        if action_type == "forward":
            cmd.linear.x = 0.3  # Forward speed
            cmd.angular.z = 0.0
        elif action_type == "backward":
            cmd.linear.x = -0.3  # Backward speed  
            cmd.angular.z = 0.0
        elif action_type == "left":
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5  # Turn left
        elif action_type == "right":
            cmd.linear.x = 0.0
            cmd.angular.z = -0.5  # Turn right
        elif action_type == "stop":
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            
        # Publish the movement command
        self.cmd_vel_publisher.publish(cmd)
        self.get_logger().info(f'✅ Published cmd_vel for {action_type}: linear.x={cmd.linear.x}, angular.z={cmd.angular.z}')
        
        # For non-stop actions, schedule a stop command after the duration
        if action_type != "stop":
            self.create_timer(duration, lambda: self._publish_stop_command())
    
    def _execute_gesture_action(self, action_type, action_data):
        """Execute gesture/behavior actions - placeholder for now"""
        self.get_logger().info(f'✅ Gesture action {action_type} executed (placeholder - no physical implementation yet)')
        
    def _publish_stop_command(self):
        """Publish a stop command"""
        from geometry_msgs.msg import Twist
        cmd = Twist()  # All zeros = stop
        self.cmd_vel_publisher.publish(cmd)
        self.get_logger().info('✅ Published stop command')
        
        # Log the action mapping for debugging
        self.get_logger().info(f'Action mapping: {action_type} -> {actions_dict.get(action_type, "NOT_FOUND")}')
        self.get_logger().info(f'Populated PicarActions.actions_dict.')

        # Execute the action if it exists in the dictionary
        if action_type in actions_dict:
            try:
                self.get_logger().info(f'Executing PicarActions method for: {action_type}')
                
                # Actions that need parameters
                if action_type in ["forward", "backward"]:
                    distance = action_data.get('distance_cm', 20)
                    speed = action_data.get('speed', None)
                    actions_dict[action_type](distance, speed)
                else:
                    # Actions that don't need parameters
                    actions_dict[action_type]()
                
                # Ensure servos are reset to straight position after action execution
                # This prevents wheels from staying skewed after actions like turn_left_in_place, etc.
                self.get_logger().info(f'Resetting servos to straight position after action: {action_type}')
                self.picar_actions.initialize_servos()
                    
            except Exception as e:
                self.get_logger().error(f'Error executing action {action_type}: {e}')
                # Even if action failed, try to reset servos for safety
                try:
                    self.picar_actions.initialize_servos()
                    self.get_logger().info('Servos reset to safe position after action error')
                except Exception as servo_error:
                    self.get_logger().error(f'Failed to reset servos after action error: {servo_error}')
        else:
            self.get_logger().warning(f'Unknown action type: {action_type}')
    
    
    def stop_robot(self):
        """Emergency stop the robot"""
        self.get_logger().warning("Emergency stop the robot")
        try:
            # Only publish if the context is still valid
            if rclpy.ok():
                cmd = Twist()
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.cmd_vel_publisher.publish(cmd)
        except Exception as e:
            # Ignore RCL context errors during shutdown
            pass
        
        # Also stop hardware if available
        if self.car is not None:
            try:
                self.car.stop()
                # Reset servos to straight position during emergency stop
                if hasattr(self, 'picar_actions') and self.picar_actions is not None:
                    self.picar_actions.initialize_servos()
                    self.get_logger().info("Servos reset to straight position during emergency stop")
            except Exception as e:
                # Use print instead of logger during shutdown to avoid RCL errors
                self.get_logger().warning("Robot emergency stop FAILED.")
        
        # Use print instead of logger during shutdown to avoid RCL errors
        self.get_logger().warning("Robot emergency stopped.")


def main(args=None):
    try:
        rclpy.init(args=args)
        
        # Validate ROS2 context before creating node
        if not rclpy.ok():
            print("ERROR: ROS2 context is not valid - cannot start NavigationNode")
            return 1
        
        navigation_node = NavigationNode()
        
        try:
            rclpy.spin(navigation_node)
        except KeyboardInterrupt:
            pass
        except Exception as e:
            print(f"ERROR: NavigationNode crashed: {e}")
            return 1
        finally:
            # Safe shutdown with context validation
            try:
                navigation_node.stop_robot()
                if rclpy.ok():
                    navigation_node.get_logger().info('Shutting down Navigation Node')
                    navigation_node.destroy_node()
                else:
                    print('Shutting down Navigation Node (context invalid)')
            except Exception as e:
                print(f'Error during shutdown: {e}')
            
            try:
                if rclpy.ok():
                    rclpy.shutdown()
            except Exception as e:
                print(f'Error during RCL shutdown: {e}')
        
        return 0
        
    except Exception as e:
        print(f"FATAL ERROR: Failed to initialize NavigationNode: {e}")
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except:
            pass
        return 1


if __name__ == '__main__':
    main()