#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from std_msgs.msg import String
from nevil_interfaces_ai_msgs.msg import TextCommand, TextResponse
from nevil_interfaces_ai_msgs.action import ProcessDialog
from dotenv import load_dotenv

class AIInterfaceNode(Node):
    def __init__(self):
        super().__init__('ai_interface')
        
        # Load environment variables from .env file with absolute path
        # Try multiple possible locations for the .env file
        possible_env_paths = [
            # Current working directory (when launched from project root)
            os.path.join(os.getcwd(), '.env'),
            # Project root relative to script location
            os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', '.env')),
            # Home directory
            os.path.join(os.path.expanduser('~'), 'Nevil-picar-v2', '.env'),
            # Absolute path to known project location
            '/home/dan/Nevil-picar-v2/.env'
        ]
        
        dotenv_loaded = False
        for dotenv_path in possible_env_paths:
            if os.path.exists(dotenv_path):
                self.get_logger().info(f'Loading .env file from: {dotenv_path}')
                load_dotenv(dotenv_path=dotenv_path)
                dotenv_loaded = True
                break
        
        if not dotenv_loaded:
            self.get_logger().warning(f'Could not find .env file in any of these locations: {possible_env_paths}')
        
        # Publishers
        self.text_response_pub = self.create_publisher(TextResponse, '/nevil/text_response', 10)
        self.status_pub = self.create_publisher(String, 'ai_status', 10)
        
        # Subscribers
        self.text_command_sub = self.create_subscription(
            TextCommand,
            '/nevil/text_command',
            self.handle_text_command,
            10
        )
        
        # Action server for more complex dialog processing
        self._action_server = ActionServer(
            self,
            ProcessDialog,
            'process_dialog',
            self.execute_dialog_callback
        )
        
        # OpenAI API configuration
        self.openai_api_key = os.getenv('OPENAI_API_KEY')
        self.openai_assistant_id = os.getenv('OPENAI_ASSISTANT_ID')
        self.get_logger().info(f'OpenAI Assistant: {self.openai_assistant_id}')
        
        if not self.openai_api_key:
            self.get_logger().error('OPENAI_API_KEY environment variable not set in .env file')
            # For testing without OpenAI, we'll still allow the node to run
        
        if not self.openai_assistant_id:
            self.get_logger().warning('OPENAI_ASSISTANT_ID not set, using basic chat completion')
        else:
            self.get_logger().info(f'Using OpenAI Assistant: {self.openai_assistant_id}')
        
        # Message deduplication
        self.processed_commands = set()
        self.max_command_history = 100  # Keep last 100 command IDs
        
        self.get_logger().info('AI Interface Node initialized')
    
    def handle_text_command(self, msg):
        """Handle incoming text commands from speech recognition"""
        self.get_logger().info(f'Received text command: {msg.command_text}')
        
        # Check for duplicate messages using command_id
        if hasattr(msg, 'command_id') and msg.command_id:
            if msg.command_id in self.processed_commands:
                self.get_logger().debug(f'Ignoring duplicate command: {msg.command_id}')
                return
            
            # Add to processed commands and limit history size
            self.processed_commands.add(msg.command_id)
            if len(self.processed_commands) > self.max_command_history:
                # Remove oldest entries (convert to list, remove first half, convert back)
                commands_list = list(self.processed_commands)
                self.processed_commands = set(commands_list[self.max_command_history//2:])
        
        # Process the command with OpenAI or use a fallback
        response = self.process_with_openai(msg.command_text)
        
        # Publish the response
        response_msg = TextResponse()
        response_msg.header.stamp = self.get_clock().now().to_msg()
        response_msg.response_text = response
        response_msg.response_type = 'answer'
        response_msg.status = 0
        response_msg.command_id = msg.command_id
        response_msg.context_id = msg.context_id
        self.text_response_pub.publish(response_msg)
        
        # Also publish status update
        status_msg = String()
        status_msg.data = 'AI system processed command'
        self.status_pub.publish(status_msg)
    
    def execute_dialog_callback(self, goal_handle):
        """Handle dialog processing action requests"""
        goal = goal_handle.request
        self.get_logger().info(f'Processing dialog: {goal.initial_utterance}')
        
        # Initialize feedback
        feedback_msg = ProcessDialog.Feedback()
        feedback_msg.current_state = "processing"
        feedback_msg.last_utterance = goal.initial_utterance
        feedback_msg.turn_count = 1
        feedback_msg.elapsed_time = 0.0
        
        # Send initial feedback
        goal_handle.publish_feedback(feedback_msg)
        
        # Process the dialog with OpenAI
        response = self.process_with_openai(goal.initial_utterance)
        
        # Update feedback
        feedback_msg.current_state = "completed"
        feedback_msg.last_utterance = response
        feedback_msg.turn_count = 2
        feedback_msg.elapsed_time = 1.0  # Approximate
        goal_handle.publish_feedback(feedback_msg)
        
        # Initialize result
        result = ProcessDialog.Result()
        
        # Set result fields
        result.success = True
        result.message = "Dialog processed successfully"
        result.final_state = "completed"
        result.dialog_summary = f"User: {goal.initial_utterance}, AI: {response}"
        result.actions_taken = ["processed_text"]
        
        # Publish the response for TTS
        response_msg = TextResponse()
        response_msg.header.stamp = self.get_clock().now().to_msg()
        response_msg.response_text = response
        response_msg.response_type = 'answer'
        response_msg.status = 0
        response_msg.command_id = ''
        response_msg.context_id = ''
        self.text_response_pub.publish(response_msg)
        
        goal_handle.succeed(result)
        return result
    
    def process_with_openai(self, text):
        """Process text with OpenAI API and return response"""
        try:
            if not self.openai_api_key:
                self.get_logger().warning('OpenAI API key not set, using fallback response')
                return self.generate_fallback_response(text)
            
            # Import here to avoid dependency issues if OpenAI is not installed
            from openai import OpenAI
            
            # Initialize the client
            client = OpenAI(api_key=self.openai_api_key)
            
            # Use Assistant API if assistant ID is available
            if self.openai_assistant_id:
                return self.process_with_assistant(client, text)
            else:
                # Fallback to basic chat completion
                response = client.chat.completions.create(
                    model="gpt-3.5-turbo",
                    messages=[
                        {"role": "system", "content": "You are a helpful assistant for a robot named Nevil. Keep responses concise and clear."},
                        {"role": "user", "content": text}
                    ],
                    max_tokens=150  # Limit response length for faster processing
                )
                
                # Extract and return the response text
                response_text = response.choices[0].message.content
                self.get_logger().info(f'OpenAI response: {response_text}')
                return response_text
        
        except ImportError:
            self.get_logger().error('OpenAI package not installed')
            return self.generate_fallback_response(text)
        except Exception as e:
            self.get_logger().error(f'Error calling OpenAI API: {str(e)}')
            return self.generate_fallback_response(text)
    
    def process_with_assistant(self, client, text):
        """Process text using OpenAI Assistant API"""
        try:
            # Create a thread for the conversation
            thread = client.beta.threads.create()
            
            # Add the user message to the thread
            client.beta.threads.messages.create(
                thread_id=thread.id,
                role="user",
                content=text
            )
            
            # Run the assistant
            run = client.beta.threads.runs.create(
                thread_id=thread.id,
                assistant_id=self.openai_assistant_id
            )
            
            # Wait for the run to complete
            import time
            while run.status in ['queued', 'in_progress']:
                time.sleep(0.5)
                run = client.beta.threads.runs.retrieve(
                    thread_id=thread.id,
                    run_id=run.id
                )
            
            if run.status == 'completed':
                # Get the assistant's response
                messages = client.beta.threads.messages.list(
                    thread_id=thread.id
                )
                
                # Get the latest assistant message
                for message in messages.data:
                    if message.role == 'assistant':
                        response_text = message.content[0].text.value
                        self.get_logger().info(f'OpenAI Assistant response: {response_text}')
                        return response_text
                
                return "I'm sorry, I couldn't generate a response."
            else:
                self.get_logger().error(f'Assistant run failed with status: {run.status}')
                return self.generate_fallback_response(text)
                
        except Exception as e:
            self.get_logger().error(f'Error with OpenAI Assistant: {str(e)}')
            return self.generate_fallback_response(text)
    
    def generate_fallback_response(self, text):
        """Generate a fallback response when OpenAI is unavailable"""
        # Simple keyword-based responses
        text_lower = text.lower()
        
        if 'hello' in text_lower or 'hi' in text_lower:
            return "Hello! I'm Nevil. How can I help you?"
        
        if 'name' in text_lower:
            return "My name is Nevil. I'm a robot assistant."
        
        if 'help' in text_lower:
            return "I can help you with information, navigation, or just have a conversation."
        
        if 'thank' in text_lower:
            return "You're welcome! Is there anything else I can help with?"
        
        if 'move' in text_lower or 'go' in text_lower:
            return "I understand you want me to move, but I need more specific instructions."
        
        if 'stop' in text_lower:
            return "Stopping now."
        
        # Default response
        return "I'm not sure how to respond to that. Could you rephrase your request?"

def main(args=None):
    rclpy.init(args=args)
    
    ai_interface_node = AIInterfaceNode()
    
    try:
        rclpy.spin(ai_interface_node)
    except KeyboardInterrupt:
        pass
    finally:
        ai_interface_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
