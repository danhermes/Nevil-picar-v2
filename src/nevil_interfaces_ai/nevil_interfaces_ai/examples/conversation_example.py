#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import uuid
import time
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import Bool
from nevil_interfaces_ai.msg import TextCommand, TextResponse, DialogState
from nevil_interfaces_ai.action import ProcessDialog

class ConversationExample(Node):
    """
    Example node demonstrating multi-turn conversations with the robot.
    
    This example shows how to:
    1. Start a conversation using the ProcessDialog action
    2. Monitor dialog state and feedback
    3. Participate in a multi-turn conversation
    4. End a conversation gracefully
    """
    
    def __init__(self):
        super().__init__('conversation_example')
        
        # Create callback groups
        self.cb_group_subs = MutuallyExclusiveCallbackGroup()
        self.cb_group_actions = MutuallyExclusiveCallbackGroup()
        
        # Create publishers
        self.text_command_pub = self.create_publisher(
            TextCommand,
            '/nevil/text_command',
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
        
        self.dialog_state_sub = self.create_subscription(
            DialogState,
            '/nevil/dialog_state',
            self.dialog_state_callback,
            10,
            callback_group=self.cb_group_subs
        )
        
        # Create action clients
        self._process_dialog_client = ActionClient(
            self,
            ProcessDialog,
            '/nevil/process_dialog',
            callback_group=self.cb_group_actions
        )
        
        # Initialize state variables
        self.context_id = str(uuid.uuid4())
        self.current_dialog_state = 'idle'
        self.conversation_active = False
        self.last_response_time = 0.0
        self.response_received = False
        
        # Create a timer for sending follow-up commands
        self.follow_up_timer = self.create_timer(2.0, self.check_follow_up)
        
        # List of example conversation turns
        self.conversation_turns = [
            "Hello, what can you do?",
            "Can you help me navigate around obstacles?",
            "How do you detect obstacles?",
            "What's your maximum speed?",
            "Thank you, that's all for now."
        ]
        self.current_turn = 0
        
        # Wait for action server
        self.get_logger().info('Waiting for process_dialog action server...')
        self._process_dialog_client.wait_for_server()
        
        self.get_logger().info('Conversation example initialized')
        
        # Start the conversation
        self.start_conversation()
    
    def start_conversation(self):
        """Start a new conversation using the ProcessDialog action."""
        self.get_logger().info('Starting conversation')
        
        # Create the goal
        goal_msg = ProcessDialog.Goal()
        goal_msg.initial_utterance = self.conversation_turns[0]
        goal_msg.context_id = self.context_id
        goal_msg.dialog_mode = 'conversation'
        goal_msg.timeout = 120.0  # 2 minutes timeout
        
        # Send the goal
        self._send_goal_future = self._process_dialog_client.send_goal_async(
            goal_msg,
            feedback_callback=self.process_dialog_feedback_callback
        )
        self._send_goal_future.add_done_callback(self.process_dialog_goal_response_callback)
        
        # Update state
        self.conversation_active = True
        self.current_turn = 1  # Move to the next turn
        self.last_response_time = time.time()
    
    def process_dialog_goal_response_callback(self, future):
        """Handle the response to the ProcessDialog goal request."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Process dialog goal rejected')
            self.conversation_active = False
            return
        
        self.get_logger().info('Process dialog goal accepted')
        
        # Get the final result when the dialog completes
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.process_dialog_result_callback)
    
    def process_dialog_feedback_callback(self, feedback_msg):
        """Handle feedback from the ProcessDialog action."""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Dialog state: {feedback.current_state}, Turn count: {feedback.turn_count}')
        
        # Update the last utterance time when we receive feedback
        if feedback.last_utterance:
            self.last_response_time = time.time()
            self.response_received = True
    
    def process_dialog_result_callback(self, future):
        """Handle the final result of the ProcessDialog action."""
        result = future.result().result
        self.get_logger().info(f'Dialog completed: {result.success}')
        self.get_logger().info(f'Final state: {result.final_state}')
        self.get_logger().info(f'Dialog summary: {result.dialog_summary}')
        
        # Update state
        self.conversation_active = False
        
        # Cancel the follow-up timer
        self.follow_up_timer.cancel()
    
    def text_response_callback(self, msg):
        """Handle text responses from the robot."""
        if msg.context_id == self.context_id:
            self.get_logger().info(f'Received response: {msg.response_text}')
            self.last_response_time = time.time()
            self.response_received = True
    
    def dialog_state_callback(self, msg):
        """Handle dialog state updates."""
        if msg.context_id == self.context_id:
            self.current_dialog_state = msg.state
            self.get_logger().debug(f'Dialog state updated: {self.current_dialog_state}')
    
    def check_follow_up(self):
        """Check if it's time to send a follow-up command."""
        if not self.conversation_active:
            return
        
        # Check if we've received a response and enough time has passed
        current_time = time.time()
        if self.response_received and (current_time - self.last_response_time) > 5.0:
            # Reset the flag
            self.response_received = False
            
            # Send the next turn if available
            if self.current_turn < len(self.conversation_turns):
                self.send_follow_up(self.conversation_turns[self.current_turn])
                self.current_turn += 1
    
    def send_follow_up(self, text):
        """Send a follow-up command in the conversation."""
        self.get_logger().info(f'Sending follow-up: {text}')
        
        # Create and publish the command
        msg = TextCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.command_text = text
        msg.source = 'user'
        msg.command_type = 'query'
        msg.priority = 100
        msg.command_id = str(uuid.uuid4())
        msg.context_id = self.context_id
        
        self.text_command_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    
    conversation_example = ConversationExample()
    
    # Use a MultiThreadedExecutor to enable processing multiple callbacks in parallel
    executor = MultiThreadedExecutor()
    executor.add_node(conversation_example)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        conversation_example.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()