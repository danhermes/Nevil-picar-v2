#!/usr/bin/env python3

import sys
import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import json
import math
from .picar_actions import PicarActions

try:
    from .picarx import Picarx
    HARDWARE_AVAILABLE = True
    print("PiCar-X hardware imported successfully in navigation_node")
except ImportError as e:
    HARDWARE_AVAILABLE = False
    print(f"PiCar-X hardware not available in navigation_node - running in simulation mode: {e}")

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
        
        # QoS profile for navigation messages
        nav_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Initialize parameters
        self.declare_parameter('navigation_mode', 'manual')  # manual, autonomous, learning
        self.declare_parameter('max_speed', 0.5)  # m/s
        
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
        
        #def init_car(self):
                # Initialize PiCar hardware
        if HARDWARE_AVAILABLE:
            self.car = Picarx()
            self.car.SafeDistance = 30
            self.car.DangerDistance = 15
            self.speed = 30
            self.DEFAULT_HEAD_TILT = 20
            self.get_logger().info("PiCar hardware initialized")
        else:
            self.car = None
            self.get_logger().warn("Running without PiCar hardware")

        self.get_logger().info('Navigation Node initialized')
        
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
            self.get_logger().info('Navigation stopped due to standby mode')
    
    def navigation_loop(self):
        """Main navigation control loop"""
        if not self.navigation_active or self.current_goal is None:
            return
            
        # Here we would implement path following
        # For now, just send a simple command
        cmd = Twist()
        
        # Simple example: move forward at half speed
        nav_mode = self.get_parameter('navigation_mode').get_parameter_value().string_value
        max_speed = self.get_parameter('max_speed').get_parameter_value().double_value
        
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
        """Execute a specific action using PicarActions"""
        self.get_logger().info(f'Executing PicarActions action type: {action_type}.')
        
        # Initialize PicarActions if not already done
        if not hasattr(self, 'picar_actions'):
            self.picar_actions = PicarActions()
        
        # Actions dictionary mapping command names to PicarActions methods
        actions_dict = {
            "forward": self.picar_actions.move_forward_this_way,
            "backward": self.picar_actions.move_backward_this_way,
            "left": self.picar_actions.turn_left,
            "right": self.picar_actions.turn_right,
            "stop": self.picar_actions.stop,
            "twist_left": self.picar_actions.turn_left_in_place,
            "twist_right": self.picar_actions.turn_right_in_place,
            "shake_head": self.picar_actions.shake_head,
            "nod": self.picar_actions.nod,
            "wave_hands": self.picar_actions.wave_hands,
            "resist": self.picar_actions.resist,
            "act_cute": self.picar_actions.act_cute,
            "rub_hands": self.picar_actions.rub_hands,
            "think": self.picar_actions.think,
            "twist_body": self.picar_actions.twist_body,
            "celebrate": self.picar_actions.celebrate,
            "depressed": self.picar_actions.depressed,
            "keep_think": self.picar_actions.keep_think,
            "honk": self.picar_actions.honk,
            "start_engine": self.picar_actions.start_engine
        }
        self.get_logger().info(f'Populated PicarActions.actions_dict.')

        # Execute the action if it exists in the dictionary
        if action_type in actions_dict:
            try:
                self.get_logger().info(f'Executing PicarActions method for: {action_type}')
                
                # Actions that need parameters
                if action_type in ["forward", "backward"]:
                    distance = action_data.get('distance_cm', 20)
                    speed = action_data.get('speed', None)
                    actions_dict[action_type](self.car,distance, speed)
                else:
                    # Actions that don't need parameters
                    actions_dict[action_type](self.car)
                    
            except Exception as e:
                self.get_logger().error(f'Error executing action {action_type}: {e}')
        else:
            self.get_logger().warning(f'Unknown action type: {action_type}')
    
    
    def stop_robot(self):
        """Emergency stop the robot"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_publisher.publish(cmd)
        self.get_logger().info('Robot stopped')


def main(args=None):
    rclpy.init(args=args)
    
    navigation_node = NavigationNode()
    
    try:
        rclpy.spin(navigation_node)
    except KeyboardInterrupt:
        pass
    finally:
        navigation_node.stop_robot()
        navigation_node.get_logger().info('Shutting down Navigation Node')
        navigation_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()