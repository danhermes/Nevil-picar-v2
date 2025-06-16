#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import uuid
import time
from std_msgs.msg import String, Bool
from nevil_interfaces_ai_msgs.msg import TextCommand, TextResponse, VoiceCommand, VoiceResponse

class BasicCommandExample(Node):
    """
    Example node demonstrating basic command and control via text and voice interfaces.
    
    This example shows how to:
    1. Send text commands to the robot
    2. Receive text responses
    3. Trigger voice recognition
    4. Receive voice commands
    """
    
    def __init__(self):
        super().__init__('basic_command_example')
        
        # Create publishers
        self.text_command_pub = self.create_publisher(
            TextCommand,
            '/nevil/text_command',
            10
        )
        
        self.listen_trigger_pub = self.create_publisher(
            Bool,
            '/nevil/listen_trigger',
            10
        )
        
        # Create subscribers
        self.text_response_sub = self.create_subscription(
            TextResponse,
            '/nevil/text_response',
            self.text_response_callback,
            10
        )
        
        self.voice_command_sub = self.create_subscription(
            VoiceCommand,
            '/nevil/voice_command',
            self.voice_command_callback,
            10
        )
        
        # Initialize state variables
        self.context_id = str(uuid.uuid4())
        self.received_responses = []
        self.received_voice_commands = []
        
        # Create a timer for sending commands
        self.command_timer = self.create_timer(5.0, self.send_command)
        self.command_index = 0
        
        # List of example commands to send
        self.example_commands = [
            "Hello, what can you do?",
            "Move forward 1 meter",
            "Turn left",
            "What is your battery level?",
            "Stop"
        ]
        
        self.get_logger().info('Basic command example initialized')
    
    def send_command(self):
        """Send a text command from the example list."""
        if self.command_index >= len(self.example_commands):
            self.command_timer.cancel()
            self.get_logger().info('All example commands sent')
            
            # After sending all text commands, trigger voice recognition
            self.get_logger().info('Now triggering voice recognition...')
            self.trigger_voice_recognition()
            return
        
        command_text = self.example_commands[self.command_index]
        self.send_text_command(command_text)
        self.command_index += 1
    
    def send_text_command(self, command_text):
        """Send a text command to the robot."""
        msg = TextCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.command_text = command_text
        msg.source = 'user'
        msg.command_type = 'query'  # Default type, will be determined by processor
        msg.priority = 100  # Medium priority
        msg.command_id = str(uuid.uuid4())
        msg.context_id = self.context_id
        
        self.text_command_pub.publish(msg)
        self.get_logger().info(f'Sent text command: {command_text}')
    
    def trigger_voice_recognition(self):
        """Trigger voice recognition."""
        msg = Bool()
        msg.data = True
        self.listen_trigger_pub.publish(msg)
        self.get_logger().info('Triggered voice recognition')
    
    def text_response_callback(self, msg):
        """Handle text responses from the robot."""
        self.get_logger().info(f'Received text response: {msg.response_text}')
        self.received_responses.append(msg)
    
    def voice_command_callback(self, msg):
        """Handle voice commands recognized by the robot."""
        self.get_logger().info(f'Received voice command: {msg.recognized_text}')
        self.received_voice_commands.append(msg)
        
        # Echo back the voice command as a text command
        self.send_text_command(msg.recognized_text)


def main(args=None):
    rclpy.init(args=args)
    
    example_node = BasicCommandExample()
    
    try:
        rclpy.spin(example_node)
    except KeyboardInterrupt:
        pass
    finally:
        example_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()