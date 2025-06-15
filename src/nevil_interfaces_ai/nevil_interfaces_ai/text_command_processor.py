#!/usr/bin/env python3

import json
import uuid
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient

from std_msgs.msg import String
from nevil_interfaces.action import NavigateToPoint, PerformBehavior
from nevil_interfaces.msg import SystemStatus
from nevil_interfaces_ai.msg import TextCommand, TextResponse, DialogState
from nevil_interfaces_ai.srv import TranslateCommand, QueryCapabilities

class TextCommandProcessor(Node):
    """
    Text command processor node for Nevil-picar v2.0.
    
    This node processes text commands from various sources (user input, API, etc.)
    and translates them into robot actions using natural language processing.
    """
    
    def __init__(self):
        super().__init__('text_command_processor')
        
        # Create callback groups
        self.cb_group_subs = MutuallyExclusiveCallbackGroup()
        self.cb_group_services = MutuallyExclusiveCallbackGroup()
        self.cb_group_actions = MutuallyExclusiveCallbackGroup()
        
        # Create publishers
        self.text_response_pub = self.create_publisher(
            TextResponse,
            '/nevil/text_response',
            10
        )
        
        self.dialog_state_pub = self.create_publisher(
            DialogState,
            '/nevil/dialog_state',
            10
        )
        
        # Create subscribers
        self.text_command_sub = self.create_subscription(
            TextCommand,
            '/nevil/text_command',
            self.text_command_callback,
            10,
            callback_group=self.cb_group_subs
        )
        
        self.system_status_sub = self.create_subscription(
            SystemStatus,
            '/system_status',
            self.system_status_callback,
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
        
        self.behavior_client = ActionClient(
            self,
            PerformBehavior,
            '/perform_behavior',
            callback_group=self.cb_group_actions
        )
        
        # Initialize state variables
        self.system_status = None
        self.dialog_contexts = {}  # Dictionary to store dialog contexts by ID
        self.command_history = {}  # Dictionary to store command history by ID
        
        # Wait for services
        self.get_logger().info('Waiting for services...')
        self.translate_command_client.wait_for_service()
        
        # Wait for action servers
        self.get_logger().info('Waiting for action servers...')
        self.navigate_client.wait_for_server()
        self.behavior_client.wait_for_server()
        
        self.get_logger().info('Text command processor initialized')
    
    def text_command_callback(self, msg):
        """Process incoming text commands."""
        self.get_logger().info(f'Received text command: {msg.command_text}')
        
        # Update dialog state
        self.update_dialog_state(msg.context_id, 'processing', msg.command_id)
        
        # Translate the natural language command to a structured command
        response = self.translate_command(msg.command_text, msg.context_id)
        
        if not response.success:
            self.send_error_response(msg, "Failed to translate command: " + response.message)
            return
        
        # Process the command based on its type
        if response.command_type == 'navigation':
            self.process_navigation_command(msg, response)
        elif response.command_type == 'behavior':
            self.process_behavior_command(msg, response)
        elif response.command_type == 'query':
            self.process_query_command(msg, response)
        elif response.command_type == 'system':
            self.process_system_command(msg, response)
        else:
            self.send_error_response(msg, f"Unknown command type: {response.command_type}")
    
    def system_status_callback(self, msg):
        """Store the latest system status."""
        self.system_status = msg
    
    def translate_command(self, command_text, context_id):
        """Translate natural language command to structured command."""
        request = TranslateCommand.Request()
        request.natural_language_command = command_text
        request.context_id = context_id
        
        future = self.translate_command_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        return future.result()
    
    def process_navigation_command(self, command_msg, translated_cmd):
        """Process navigation commands."""
        try:
            # Parse parameters
            params = json.loads(translated_cmd.parameters)
            
            # Create navigation goal
            goal_msg = NavigateToPoint.Goal()
            
            # Set goal parameters based on the command
            if translated_cmd.command_action == 'move_forward':
                distance = params.get('distance', 0.5)  # Default to 0.5 meters
                goal_msg.target_pose.pose.position.x = distance
            elif translated_cmd.command_action == 'move_backward':
                distance = params.get('distance', 0.5)  # Default to 0.5 meters
                goal_msg.target_pose.pose.position.x = -distance
            elif translated_cmd.command_action == 'turn_left':
                goal_msg.angular_velocity = params.get('angle', 90.0)  # Default to 90 degrees
            elif translated_cmd.command_action == 'turn_right':
                goal_msg.angular_velocity = -params.get('angle', 90.0)  # Default to 90 degrees
            else:
                self.send_error_response(command_msg, f"Unknown navigation action: {translated_cmd.command_action}")
                return
            
            # Set common parameters
            goal_msg.linear_velocity = params.get('speed', 0.5)  # Default to 0.5 m/s
            goal_msg.avoid_obstacles = params.get('avoid_obstacles', True)
            
            # Send the goal
            self.get_logger().info(f'Sending navigation goal: {translated_cmd.command_action}')
            self.navigate_client.send_goal_async(
                goal_msg,
                feedback_callback=lambda feedback: self.navigation_feedback_callback(feedback, command_msg)
            ).add_done_callback(
                lambda future: self.navigation_goal_response_callback(future, command_msg)
            )
            
            # Send initial response
            self.send_text_response(
                command_msg,
                f"Executing navigation command: {translated_cmd.command_action}",
                'confirmation',
                0
            )
            
        except Exception as e:
            self.get_logger().error(f'Error processing navigation command: {e}')
            self.send_error_response(command_msg, f"Error processing navigation command: {str(e)}")
    
    def process_behavior_command(self, command_msg, translated_cmd):
        """Process behavior commands."""
        try:
            # Parse parameters
            params = json.loads(translated_cmd.parameters)
            
            # Create behavior goal
            goal_msg = PerformBehavior.Goal()
            goal_msg.behavior_name = translated_cmd.command_action
            goal_msg.duration = params.get('duration', 5.0)  # Default to 5 seconds
            
            # Add any additional parameters as a JSON string
            goal_msg.parameters = json.dumps(params)
            
            # Send the goal
            self.get_logger().info(f'Sending behavior goal: {translated_cmd.command_action}')
            self.behavior_client.send_goal_async(
                goal_msg,
                feedback_callback=lambda feedback: self.behavior_feedback_callback(feedback, command_msg)
            ).add_done_callback(
                lambda future: self.behavior_goal_response_callback(future, command_msg)
            )
            
            # Send initial response
            self.send_text_response(
                command_msg,
                f"Executing behavior: {translated_cmd.command_action}",
                'confirmation',
                0
            )
            
        except Exception as e:
            self.get_logger().error(f'Error processing behavior command: {e}')
            self.send_error_response(command_msg, f"Error processing behavior command: {str(e)}")
    
    def process_query_command(self, command_msg, translated_cmd):
        """Process query commands."""
        try:
            # Handle different types of queries
            if translated_cmd.command_action == 'get_status':
                if self.system_status is None:
                    self.send_error_response(command_msg, "System status not available")
                    return
                
                # Format system status as a response
                response_text = (
                    f"System status: {self.system_status.mode}\n"
                    f"Battery: {self.system_status.battery_level * 100:.1f}%\n"
                    f"System health: {'OK' if self.system_status.system_ok else 'Error'}\n"
                )
                
                if self.system_status.has_errors:
                    response_text += f"Errors: {', '.join(self.system_status.error_messages)}\n"
                
                self.send_text_response(command_msg, response_text, 'answer', 0)
                
            elif translated_cmd.command_action == 'get_capabilities':
                # This would typically call the QueryCapabilities service
                # For now, we'll just return a simple response
                response_text = (
                    "I can help you with:\n"
                    "- Navigation (move forward/backward, turn left/right)\n"
                    "- Behaviors (wave, nod, shake head)\n"
                    "- Queries about my status and capabilities\n"
                )
                
                self.send_text_response(command_msg, response_text, 'answer', 0)
                
            else:
                self.send_error_response(command_msg, f"Unknown query action: {translated_cmd.command_action}")
                
        except Exception as e:
            self.get_logger().error(f'Error processing query command: {e}')
            self.send_error_response(command_msg, f"Error processing query command: {str(e)}")
    
    def process_system_command(self, command_msg, translated_cmd):
        """Process system commands."""
        try:
            # Handle different types of system commands
            if translated_cmd.command_action == 'shutdown':
                response_text = "Initiating shutdown sequence. Goodbye!"
                self.send_text_response(command_msg, response_text, 'confirmation', 0)
                # In a real implementation, this would trigger a system shutdown
                
            elif translated_cmd.command_action == 'restart':
                response_text = "Restarting system. I'll be back in a moment."
                self.send_text_response(command_msg, response_text, 'confirmation', 0)
                # In a real implementation, this would trigger a system restart
                
            else:
                self.send_error_response(command_msg, f"Unknown system action: {translated_cmd.command_action}")
                
        except Exception as e:
            self.get_logger().error(f'Error processing system command: {e}')
            self.send_error_response(command_msg, f"Error processing system command: {str(e)}")
    
    def navigation_feedback_callback(self, feedback_msg, command_msg):
        """Handle navigation action feedback."""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Navigation feedback: {feedback}')
        
        # Update dialog state with progress information
        self.update_dialog_state(command_msg.context_id, 'executing', command_msg.command_id)
    
    def navigation_goal_response_callback(self, future, command_msg):
        """Handle navigation goal response."""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().warn('Navigation goal rejected')
            self.send_error_response(command_msg, "Navigation goal rejected")
            return
        
        self.get_logger().info('Navigation goal accepted')
        
        # Get the result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda future: self.navigation_result_callback(future, command_msg)
        )
    
    def navigation_result_callback(self, future, command_msg):
        """Handle navigation action result."""
        result = future.result().result
        
        if result.success:
            self.get_logger().info('Navigation completed successfully')
            self.send_text_response(
                command_msg,
                "Navigation completed successfully",
                'status',
                0
            )
        else:
            self.get_logger().warn(f'Navigation failed: {result.message}')
            self.send_error_response(command_msg, f"Navigation failed: {result.message}")
        
        # Update dialog state
        self.update_dialog_state(command_msg.context_id, 'idle', command_msg.command_id)
    
    def behavior_feedback_callback(self, feedback_msg, command_msg):
        """Handle behavior action feedback."""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Behavior feedback: {feedback}')
        
        # Update dialog state with progress information
        self.update_dialog_state(command_msg.context_id, 'executing', command_msg.command_id)
    
    def behavior_goal_response_callback(self, future, command_msg):
        """Handle behavior goal response."""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().warn('Behavior goal rejected')
            self.send_error_response(command_msg, "Behavior goal rejected")
            return
        
        self.get_logger().info('Behavior goal accepted')
        
        # Get the result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda future: self.behavior_result_callback(future, command_msg)
        )
    
    def behavior_result_callback(self, future, command_msg):
        """Handle behavior action result."""
        result = future.result().result
        
        if result.success:
            self.get_logger().info('Behavior completed successfully')
            self.send_text_response(
                command_msg,
                "Behavior completed successfully",
                'status',
                0
            )
        else:
            self.get_logger().warn(f'Behavior failed: {result.message}')
            self.send_error_response(command_msg, f"Behavior failed: {result.message}")
        
        # Update dialog state
        self.update_dialog_state(command_msg.context_id, 'idle', command_msg.command_id)
    
    def send_text_response(self, command_msg, response_text, response_type, status):
        """Send a text response."""
        response = TextResponse()
        response.header.stamp = self.get_clock().now().to_msg()
        response.response_text = response_text
        response.response_type = response_type
        response.status = status
        response.command_id = command_msg.command_id
        response.context_id = command_msg.context_id
        
        self.text_response_pub.publish(response)
        self.get_logger().info(f'Sent response: {response_text}')
    
    def send_error_response(self, command_msg, error_message):
        """Send an error response."""
        self.send_text_response(command_msg, error_message, 'error', 1)
        
        # Update dialog state
        self.update_dialog_state(command_msg.context_id, 'error', command_msg.command_id)
    
    def update_dialog_state(self, context_id, state, command_id):
        """Update and publish the dialog state."""
        # Create a new context if it doesn't exist
        if context_id not in self.dialog_contexts:
            self.dialog_contexts[context_id] = {
                'state': 'idle',
                'mode': 'command',
                'dialog_history': [],
                'turn_count': 0,
                'last_command_id': '',
                'last_response_id': ''
            }
        
        # Update the context
        context = self.dialog_contexts[context_id]
        context['state'] = state
        context['last_command_id'] = command_id
        
        # Create and publish the dialog state message
        dialog_state = DialogState()
        dialog_state.header.stamp = self.get_clock().now().to_msg()
        dialog_state.context_id = context_id
        dialog_state.state = state
        dialog_state.mode = context['mode']
        dialog_state.dialog_history = json.dumps(context['dialog_history'])
        dialog_state.last_command_id = command_id
        dialog_state.turn_count = context['turn_count']
        
        self.dialog_state_pub.publish(dialog_state)
        self.get_logger().debug(f'Published dialog state: {state}')


def main(args=None):
    rclpy.init(args=args)
    
    text_command_processor = TextCommandProcessor()
    
    # Use a MultiThreadedExecutor to enable processing multiple callbacks in parallel
    executor = MultiThreadedExecutor()
    executor.add_node(text_command_processor)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        text_command_processor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()