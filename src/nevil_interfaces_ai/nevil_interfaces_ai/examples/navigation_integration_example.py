#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import uuid
import time
import json
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped
from nevil_interfaces.action import NavigateToPoint
from nevil_interfaces_ai_msgs.msg import TextCommand, TextResponse
from nevil_interfaces_ai_msgs.srv import TranslateCommand
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

class NavigationIntegrationExample(Node):
    """
    Example node demonstrating integration between text commands and navigation.
    
    This example shows how to:
    1. Translate natural language commands to navigation actions
    2. Execute navigation actions
    3. Provide feedback on navigation progress
    4. Handle navigation errors
    """
    
    def __init__(self):
        super().__init__('navigation_integration_example')
        
        # Create callback groups
        self.cb_group_subs = MutuallyExclusiveCallbackGroup()
        self.cb_group_services = MutuallyExclusiveCallbackGroup()
        self.cb_group_actions = MutuallyExclusiveCallbackGroup()
        
        # Create publishers
        self.text_command_pub = self.create_publisher(
            TextCommand,
            '/nevil/text_command',
            10
        )
        
        self.text_response_pub = self.create_publisher(
            TextResponse,
            '/nevil/text_response',
            10
        )
        
        # Create subscribers
        self.text_response_sub = self.create_subscription(
            TextResponse,
            '/nevil/text_response',
            self.text_response_callback,
            10,
            callback_group=self.cb_group_subs
        )
        
        # Create service clients
        self.translate_command_client = self.create_client(
            TranslateCommand,
            '/nevil/translate_command',
            callback_group=self.cb_group_services
        )
        
        # Create action clients
        self.navigate_client = ActionClient(
            self,
            NavigateToPoint,
            '/navigate_to_point',
            callback_group=self.cb_group_actions
        )
        
        # Initialize state variables
        self.context_id = str(uuid.uuid4())
        self.current_command_id = None
        self.navigation_in_progress = False
        
        # Create a timer for sending commands
        self.command_timer = self.create_timer(10.0, self.send_command)
        self.command_index = 0
        
        # List of example navigation commands to send
        self.example_commands = [
            "Move forward 1 meter",
            "Turn left 90 degrees",
            "Move forward 0.5 meters",
            "Turn right 45 degrees",
            "Move backward 0.3 meters"
        ]
        
        # Wait for services and action servers
        self.get_logger().info('Waiting for services and action servers...')
        self.translate_command_client.wait_for_service()
        self.navigate_client.wait_for_server()
        
        self.get_logger().info('Navigation integration example initialized')
    
    def send_command(self):
        """Send a navigation command from the example list."""
        if self.navigation_in_progress:
            self.get_logger().info('Navigation in progress, waiting...')
            return
            
        if self.command_index >= len(self.example_commands):
            self.command_timer.cancel()
            self.get_logger().info('All example commands sent')
            return
        
        command_text = self.example_commands[self.command_index]
        self.process_navigation_command(command_text)
        self.command_index += 1
    
    def process_navigation_command(self, command_text):
        """Process a navigation command by translating it and executing it."""
        self.get_logger().info(f'Processing navigation command: {command_text}')
        
        # First, publish the command as a TextCommand
        self.current_command_id = str(uuid.uuid4())
        self.publish_text_command(command_text)
        
        # Then, translate the command
        self.translate_and_execute(command_text)
    
    def publish_text_command(self, command_text):
        """Publish a text command."""
        msg = TextCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.command_text = command_text
        msg.source = 'user'
        msg.command_type = 'navigation'
        msg.priority = 100
        msg.command_id = self.current_command_id
        msg.context_id = self.context_id
        
        self.text_command_pub.publish(msg)
        self.get_logger().info(f'Published text command: {command_text}')
    
    def translate_and_execute(self, command_text):
        """Translate a natural language command and execute the resulting navigation action."""
        # Create the request
        request = TranslateCommand.Request()
        request.natural_language_command = command_text
        request.context_id = self.context_id
        
        # Call the service
        future = self.translate_command_client.call_async(request)
        future.add_done_callback(self.translation_callback)
    
    def translation_callback(self, future):
        """Handle the translation response and execute the navigation action."""
        try:
            response = future.result()
            
            if not response.success:
                self.get_logger().error(f'Translation failed: {response.message}')
                self.publish_text_response(f"I couldn't understand that navigation command: {response.message}", 'error', 1)
                return
            
            self.get_logger().info(f'Translated command: {response.command_type} - {response.command_action}')
            
            # Check if this is a navigation command
            if response.command_type != 'navigation':
                self.get_logger().warn(f'Not a navigation command: {response.command_type}')
                self.publish_text_response("That's not a navigation command.", 'error', 1)
                return
            
            # Parse parameters
            params = json.loads(response.parameters)
            
            # Execute the navigation action
            self.execute_navigation_action(response.command_action, params)
            
        except Exception as e:
            self.get_logger().error(f'Error in translation callback: {e}')
            self.publish_text_response(f"Error processing navigation command: {str(e)}", 'error', 1)
    
    def execute_navigation_action(self, action, params):
        """Execute a navigation action."""
        self.get_logger().info(f'Executing navigation action: {action} with params: {params}')
        
        # Create the goal message
        goal_msg = NavigateToPoint.Goal()
        
        # Set goal parameters based on the action
        if action == 'move_forward':
            distance = params.get('distance', 0.5)  # Default to 0.5 meters
            goal_msg.target_pose = PoseStamped()
            goal_msg.target_pose.header.frame_id = 'base_link'
            goal_msg.target_pose.header.stamp = self.get_clock().now().to_msg()
            goal_msg.target_pose.pose.position.x = distance
            goal_msg.target_pose.pose.position.y = 0.0
            goal_msg.target_pose.pose.position.z = 0.0
            goal_msg.target_pose.pose.orientation.w = 1.0
            
        elif action == 'move_backward':
            distance = params.get('distance', 0.5)  # Default to 0.5 meters
            goal_msg.target_pose = PoseStamped()
            goal_msg.target_pose.header.frame_id = 'base_link'
            goal_msg.target_pose.header.stamp = self.get_clock().now().to_msg()
            goal_msg.target_pose.pose.position.x = -distance
            goal_msg.target_pose.pose.position.y = 0.0
            goal_msg.target_pose.pose.position.z = 0.0
            goal_msg.target_pose.pose.orientation.w = 1.0
            
        elif action == 'turn_left':
            angle = params.get('angle', 90.0)  # Default to 90 degrees
            goal_msg.target_pose = PoseStamped()
            goal_msg.target_pose.header.frame_id = 'base_link'
            goal_msg.target_pose.header.stamp = self.get_clock().now().to_msg()
            goal_msg.target_pose.pose.position.x = 0.0
            goal_msg.target_pose.pose.position.y = 0.0
            goal_msg.target_pose.pose.position.z = 0.0
            goal_msg.target_pose.pose.orientation.w = 1.0
            goal_msg.angular_velocity = angle
            
        elif action == 'turn_right':
            angle = params.get('angle', 90.0)  # Default to 90 degrees
            goal_msg.target_pose = PoseStamped()
            goal_msg.target_pose.header.frame_id = 'base_link'
            goal_msg.target_pose.header.stamp = self.get_clock().now().to_msg()
            goal_msg.target_pose.pose.position.x = 0.0
            goal_msg.target_pose.pose.position.y = 0.0
            goal_msg.target_pose.pose.position.z = 0.0
            goal_msg.target_pose.pose.orientation.w = 1.0
            goal_msg.angular_velocity = -angle
            
        else:
            self.get_logger().error(f'Unknown navigation action: {action}')
            self.publish_text_response(f"I don't know how to {action}", 'error', 1)
            return
        
        # Set common parameters
        goal_msg.linear_velocity = params.get('speed', 0.5)  # Default to 0.5 m/s
        goal_msg.avoid_obstacles = params.get('avoid_obstacles', True)
        goal_msg.goal_tolerance = 0.05  # 5cm tolerance
        
        # Send the goal
        self.navigation_in_progress = True
        self.publish_text_response(f"Executing navigation: {action}", 'confirmation', 0)
        
        self.navigate_client.send_goal_async(
            goal_msg,
            feedback_callback=self.navigation_feedback_callback
        ).add_done_callback(self.navigation_goal_response_callback)
    
    def navigation_feedback_callback(self, feedback_msg):
        """Handle navigation action feedback."""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Navigation feedback: {feedback}')
    
    def navigation_goal_response_callback(self, future):
        """Handle navigation goal response."""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().warn('Navigation goal rejected')
            self.publish_text_response("Navigation goal rejected", 'error', 1)
            self.navigation_in_progress = False
            return
        
        self.get_logger().info('Navigation goal accepted')
        
        # Get the result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.navigation_result_callback)
    
    def navigation_result_callback(self, future):
        """Handle navigation action result."""
        result = future.result().result
        
        if result.success:
            self.get_logger().info('Navigation completed successfully')
            self.publish_text_response("Navigation completed successfully", 'status', 0)
        else:
            self.get_logger().warn(f'Navigation failed: {result.message}')
            self.publish_text_response(f"Navigation failed: {result.message}", 'error', 1)
        
        self.navigation_in_progress = False
    
    def text_response_callback(self, msg):
        """Handle text responses."""
        if msg.command_id == self.current_command_id:
            self.get_logger().info(f'Received response for current command: {msg.response_text}')
    
    def publish_text_response(self, response_text, response_type, status):
        """Publish a text response."""
        msg = TextResponse()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.response_text = response_text
        msg.response_type = response_type
        msg.status = status
        msg.command_id = self.current_command_id
        msg.context_id = self.context_id
        
        self.text_response_pub.publish(msg)
        self.get_logger().info(f'Published response: {response_text}')


def main(args=None):
    rclpy.init(args=args)
    
    example_node = NavigationIntegrationExample()
    
    # Use a MultiThreadedExecutor to enable processing multiple callbacks in parallel
    executor = MultiThreadedExecutor()
    executor.add_node(example_node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        example_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()