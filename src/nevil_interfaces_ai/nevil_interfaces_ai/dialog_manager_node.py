#!/usr/bin/env python3

import os
import uuid
import json
import threading
import time
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle

from std_msgs.msg import Bool, String
from nevil_interfaces.msg import SystemStatus
from nevil_interfaces_ai_msgs.msg import TextCommand, TextResponse, VoiceCommand, VoiceResponse, DialogState
from nevil_interfaces_ai_msgs.action import ProcessDialog
from nevil_interfaces_ai_msgs.srv import TranslateCommand

class DialogManager(Node):
    """
    Dialog manager node for Nevil-picar v2.0.
    
    This node manages conversations and dialog state, maintaining context
    across multiple interactions and coordinating between text and voice interfaces.
    """
    
    def __init__(self):
        super().__init__('dialog_manager_node')
        
        # Declare parameters
        self.declare_parameter('max_context_size', 10)  # Max number of turns to keep in context
        self.declare_parameter('idle_timeout', 60.0)    # Seconds to wait before resetting context
        self.declare_parameter('use_cloud_ai', True)    # Whether to use cloud AI for NLP
        self.declare_parameter('api_key', '')           # API key for cloud services
        
        # Get parameters
        self.max_context_size = self.get_parameter('max_context_size').value
        self.idle_timeout = self.get_parameter('idle_timeout').value
        self.use_cloud_ai = self.get_parameter('use_cloud_ai').value
        self.api_key = self.get_parameter('api_key').value
        
        # Create callback groups
        self.cb_group_subs = MutuallyExclusiveCallbackGroup()
        self.cb_group_pubs = MutuallyExclusiveCallbackGroup()
        self.cb_group_services = MutuallyExclusiveCallbackGroup()
        self.cb_group_actions = MutuallyExclusiveCallbackGroup()
        
        # Create publishers
        self.dialog_state_pub = self.create_publisher(
            DialogState,
            '/nevil/dialog_state',
            10
        )
        
        self.text_response_pub = self.create_publisher(
            TextResponse,
            '/nevil/text_response',
            10
        )
        
        self.voice_response_pub = self.create_publisher(
            VoiceResponse,
            '/nevil/voice_response',
            10
        )
        
        self.listen_trigger_pub = self.create_publisher(
            Bool,
            '/nevil/listen_trigger',
            10
        )
        
        self.text_command_pub = self.create_publisher(
            TextCommand,
            '/nevil/text_command',
            10,
            callback_group=self.cb_group_pubs
        )
        
        # Create subscribers
        self.text_command_sub = self.create_subscription(
            TextCommand,
            '/nevil/text_command',
            self.text_command_callback,
            10,
            callback_group=self.cb_group_subs
        )
        
        self.voice_command_sub = self.create_subscription(
            VoiceCommand,
            '/nevil/voice_command',
            self.voice_command_callback,
            10,
            callback_group=self.cb_group_subs
        )
        
        self.text_response_sub = self.create_subscription(
            TextResponse,
            '/nevil/text_response',
            self.text_response_callback,
            10,
            callback_group=self.cb_group_subs
        )
        
        self.speaking_status_sub = self.create_subscription(
            Bool,
            '/nevil/speaking_status',
            self.speaking_status_callback,
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
        
        # Create action servers
        self._process_dialog_action_server = ActionServer(
            self,
            ProcessDialog,
            '/nevil/process_dialog',
            execute_callback=self.execute_process_dialog,
            goal_callback=self.process_dialog_goal_callback,
            cancel_callback=self.process_dialog_cancel_callback,
            callback_group=self.cb_group_actions
        )
        
        # Initialize state variables
        self.dialog_contexts = {}  # Dictionary to store dialog contexts by ID
        self.active_dialogs = {}   # Dictionary to store active dialog goals by context ID
        self.system_status = None
        self.is_speaking = False
        
        # Start idle timeout thread
        self.stop_event = threading.Event()
        self.idle_thread = threading.Thread(target=self.idle_timeout_thread)
        self.idle_thread.daemon = True
        self.idle_thread.start()
        
        self.get_logger().info('Dialog manager initialized')
    
    def text_command_callback(self, msg):
        """Handle incoming text commands."""
        self.get_logger().info(f'Received text command: {msg.command_text}')
        
        # Get or create context
        context = self.get_or_create_context(msg.context_id)
        
        # Update context with user utterance
        self.add_to_context(context, 'user', msg.command_text)
        
        # Update dialog state
        self.update_dialog_state(context, 'processing')
        
        # If this is part of an active dialog, update the dialog
        if context['id'] in self.active_dialogs:
            goal_handle = self.active_dialogs[context['id']]
            self.publish_dialog_feedback(goal_handle, context)
        
        # Otherwise, just process the command normally
        # The response will come through the text_response_callback
    
    def voice_command_callback(self, msg):
        """Handle incoming voice commands."""
        self.get_logger().info(f'Received voice command: {msg.recognized_text}')
        
        # Get or create context
        context = self.get_or_create_context(msg.context_id)
        
        # Update context with user utterance
        self.add_to_context(context, 'user', msg.recognized_text)
        
        # Update dialog state
        self.update_dialog_state(context, 'processing')
        
        # If this is part of an active dialog, update the dialog
        if context['id'] in self.active_dialogs:
            goal_handle = self.active_dialogs[context['id']]
            self.publish_dialog_feedback(goal_handle, context)
        
        # Create and publish a text command for processing
        self.create_text_command_from_voice(msg)
    
    def text_response_callback(self, msg):
        """Handle text responses and update dialog context."""
        self.get_logger().info(f'Received text response: {msg.response_text}')
        
        # Get context
        context_id = msg.context_id
        if context_id not in self.dialog_contexts:
            self.get_logger().warn(f'Received response for unknown context: {context_id}')
            return
        
        context = self.dialog_contexts[context_id]
        
        # Update context with system response
        self.add_to_context(context, 'assistant', msg.response_text)
        
        # Update dialog state
        self.update_dialog_state(context, 'speaking')
        
        # If this is part of an active dialog, update the dialog
        if context_id in self.active_dialogs:
            goal_handle = self.active_dialogs[context_id]
            self.publish_dialog_feedback(goal_handle, context)
        
        # Create and publish a voice response
        self.create_voice_response_from_text(msg)
    
    def speaking_status_callback(self, msg):
        """Handle speaking status updates."""
        self.is_speaking = msg.data
        
        # If we've stopped speaking, update dialog state and trigger listening
        if not self.is_speaking:
            for context_id, context in self.dialog_contexts.items():
                if context['state'] == 'speaking':
                    self.update_dialog_state(context, 'listening')
                    
                    # Trigger listening
                    trigger_msg = Bool()
                    trigger_msg.data = True
                    self.listen_trigger_pub.publish(trigger_msg)
                    
                    # If this is part of an active dialog, update the dialog
                    if context_id in self.active_dialogs:
                        goal_handle = self.active_dialogs[context_id]
                        self.publish_dialog_feedback(goal_handle, context)
    
    def system_status_callback(self, msg):
        """Store the latest system status."""
        self.system_status = msg
    
    def process_dialog_goal_callback(self, goal_request):
        """Handle new dialog processing goals."""
        self.get_logger().info('Received process dialog goal')
        
        # Get or create context ID
        context_id = goal_request.context_id
        if not context_id:
            context_id = str(uuid.uuid4())
        
        # Get or create context
        context = self.get_or_create_context(context_id)
        
        # Update context with initial utterance if provided
        if goal_request.initial_utterance:
            self.add_to_context(context, 'user', goal_request.initial_utterance)
        
        # Update dialog mode
        context['mode'] = goal_request.dialog_mode
        
        # Update dialog state
        self.update_dialog_state(context, 'listening')
        
        return GoalResponse.ACCEPT
    
    def process_dialog_cancel_callback(self, goal_handle):
        """Handle cancellation of dialog processing."""
        self.get_logger().info('Received cancel request for process dialog')
        
        # Find the context ID for this goal handle
        context_id = None
        for cid, gh in self.active_dialogs.items():
            if gh.goal_id == goal_handle.goal_id:
                context_id = cid
                break
        
        if context_id:
            # Remove from active dialogs
            del self.active_dialogs[context_id]
            
            # Update dialog state
            if context_id in self.dialog_contexts:
                context = self.dialog_contexts[context_id]
                self.update_dialog_state(context, 'idle')
        
        return CancelResponse.ACCEPT
    
    def execute_process_dialog(self, goal_handle):
        """Execute dialog processing action."""
        self.get_logger().info('Executing process dialog')
        
        # Get goal
        goal = goal_handle.request
        
        # Get context
        context_id = goal.context_id
        if not context_id:
            context_id = str(uuid.uuid4())
        
        context = self.get_or_create_context(context_id)
        
        # Store goal handle
        self.active_dialogs[context_id] = goal_handle
        
        # If initial utterance is provided, create a text command
        if goal.initial_utterance:
            # Create TextCommand message
            cmd_msg = TextCommand()
            cmd_msg.header.stamp = self.get_clock().now().to_msg()
            cmd_msg.command_text = goal.initial_utterance
            cmd_msg.source = 'api'
            cmd_msg.command_type = 'query'  # Default type
            cmd_msg.priority = 100  # Medium priority
            cmd_msg.command_id = str(uuid.uuid4())
            cmd_msg.context_id = context_id
            
            # Publish the message
            self.text_command_pub.publish(cmd_msg)
        
        # Start listening
        trigger_msg = Bool()
        trigger_msg.data = True
        self.listen_trigger_pub.publish(trigger_msg)
        
        # Update dialog state
        self.update_dialog_state(context, 'listening')
        
        # Publish initial feedback
        self.publish_dialog_feedback(goal_handle, context)
        
        # Wait for dialog to complete or timeout
        start_time = time.time()
        result = ProcessDialog.Result()
        
        try:
            while (time.time() - start_time) < goal.timeout:
                # Check if goal is still active
                if not goal_handle.is_active:
                    self.get_logger().info('Goal is no longer active')
                    break
                
                # Check if dialog is complete
                if context['state'] == 'idle' and len(context['history']) > 0:
                    self.get_logger().info('Dialog completed naturally')
                    result.success = True
                    result.message = "Dialog completed successfully"
                    break
                
                # Sleep briefly to avoid busy waiting
                time.sleep(0.1)
            
            # Check for timeout
            if (time.time() - start_time) >= goal.timeout:
                self.get_logger().info('Dialog timed out')
                result.success = True
                result.message = "Dialog timed out"
            
            # Set result fields
            result.final_state = context['state']
            result.dialog_summary = self.generate_dialog_summary(context)
            result.actions_taken = self.get_actions_taken(context)
            
            # Remove from active dialogs
            if context_id in self.active_dialogs:
                del self.active_dialogs[context_id]
            
            # Update dialog state
            self.update_dialog_state(context, 'idle')
            
            # Return result
            goal_handle.succeed()
            return result
            
        except Exception as e:
            self.get_logger().error(f'Error executing process dialog: {e}')
            result.success = False
            result.message = f"Error: {str(e)}"
            
            # Remove from active dialogs
            if context_id in self.active_dialogs:
                del self.active_dialogs[context_id]
            
            # Update dialog state
            self.update_dialog_state(context, 'error')
            
            # Return result
            goal_handle.abort()
            return result
    
    def publish_dialog_feedback(self, goal_handle, context):
        """Publish feedback for an active dialog."""
        feedback_msg = ProcessDialog.Feedback()
        feedback_msg.current_state = context['state']
        
        # Get last utterance
        if len(context['history']) > 0:
            feedback_msg.last_utterance = context['history'][-1]['content']
        else:
            feedback_msg.last_utterance = ""
        
        feedback_msg.turn_count = len(context['history'])
        feedback_msg.elapsed_time = time.time() - context['start_time']
        
        goal_handle.publish_feedback(feedback_msg)
    
    def get_or_create_context(self, context_id):
        """Get an existing context or create a new one."""
        if not context_id:
            context_id = str(uuid.uuid4())
        
        if context_id not in self.dialog_contexts:
            self.dialog_contexts[context_id] = {
                'id': context_id,
                'state': 'idle',
                'mode': 'command',
                'history': [],
                'start_time': time.time(),
                'last_update_time': time.time(),
                'turn_count': 0,
                'actions_taken': []
            }
        
        return self.dialog_contexts[context_id]
    
    def add_to_context(self, context, role, content):
        """Add an utterance to the dialog context."""
        # Add to history
        context['history'].append({
            'role': role,
            'content': content,
            'timestamp': time.time()
        })
        
        # Trim history if needed
        if len(context['history']) > self.max_context_size * 2:  # Keep pairs of utterances
            context['history'] = context['history'][-self.max_context_size * 2:]
        
        # Update counters
        context['turn_count'] = len(context['history']) // 2  # Count pairs as turns
        context['last_update_time'] = time.time()
    
    def update_dialog_state(self, context, state):
        """Update the dialog state and publish it."""
        context['state'] = state
        context['last_update_time'] = time.time()
        
        # Create and publish dialog state message
        msg = DialogState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.context_id = context['id']
        msg.state = state
        msg.mode = context['mode']
        msg.dialog_history = json.dumps(context['history'])
        msg.turn_count = context['turn_count']
        
        # Add environment context if available
        if self.system_status:
            env_context = {
                'system_mode': self.system_status.mode,
                'battery_level': self.system_status.battery_level,
                'system_ok': self.system_status.system_ok
            }
            msg.environment_context = json.dumps(env_context)
        
        self.dialog_state_pub.publish(msg)
        self.get_logger().debug(f'Published dialog state: {state}')
    
    def create_text_command_from_voice(self, voice_msg):
        """Create a text command from a voice command."""
        cmd = TextCommand()
        cmd.header = voice_msg.header
        cmd.command_text = voice_msg.recognized_text
        cmd.source = 'voice'
        cmd.command_type = 'query'  # Default type, will be determined by processor
        cmd.priority = 100  # Medium priority
        cmd.command_id = voice_msg.command_id
        cmd.context_id = voice_msg.context_id
        
        self.text_command_pub.publish(cmd)
    
    def create_voice_response_from_text(self, text_msg):
        """Create a voice response from a text response."""
        resp = VoiceResponse()
        resp.header = text_msg.header
        resp.text_to_speak = text_msg.response_text
        resp.voice_id = ""  # Use default voice
        resp.speaking_rate = 1.0  # Normal rate
        resp.pitch = 1.0  # Normal pitch
        resp.volume = 1.0  # Normal volume
        resp.command_id = text_msg.command_id
        resp.context_id = text_msg.context_id
        
        self.voice_response_pub.publish(resp)
    
    def generate_dialog_summary(self, context):
        """Generate a summary of the dialog."""
        # In a real implementation, this might use an AI model to summarize
        # For now, we'll just return a simple summary
        
        num_turns = context['turn_count']
        if num_turns == 0:
            return "No dialog occurred"
        
        # Get the first user utterance and last system response
        first_user = ""
        last_system = ""
        
        for entry in context['history']:
            if entry['role'] == 'user' and not first_user:
                first_user = entry['content']
            if entry['role'] == 'assistant':
                last_system = entry['content']
        
        return f"Dialog with {num_turns} turns. Started with: '{first_user}'. Ended with: '{last_system}'"
    
    def get_actions_taken(self, context):
        """Get a list of actions taken during the dialog."""
        # In a real implementation, this would extract actions from the dialog
        # For now, we'll just return a placeholder
        return ["dialog_processed"]
    
    def idle_timeout_thread(self):
        """Thread to check for idle contexts and clean them up."""
        while not self.stop_event.is_set():
            try:
                current_time = time.time()
                
                # Check each context for timeout
                for context_id in list(self.dialog_contexts.keys()):
                    context = self.dialog_contexts[context_id]
                    
                    # Skip active dialogs
                    if context_id in self.active_dialogs:
                        continue
                    
                    # Check if context has timed out
                    if (current_time - context['last_update_time']) > self.idle_timeout:
                        self.get_logger().info(f'Context {context_id} timed out, resetting')
                        
                        # Reset context to idle state
                        self.update_dialog_state(context, 'idle')
                        
                        # Optionally, remove the context entirely
                        # del self.dialog_contexts[context_id]
                
                # Sleep to avoid busy waiting
                time.sleep(5.0)
                
            except Exception as e:
                self.get_logger().error(f'Error in idle timeout thread: {e}')
    
    def shutdown(self):
        """Clean up resources."""
        self.stop_event.set()
        if self.idle_thread.is_alive():
            self.idle_thread.join(timeout=1.0)


def main(args=None):
    rclpy.init(args=args)
    
    dialog_manager = DialogManager()
    
    # Use a MultiThreadedExecutor to enable processing multiple callbacks in parallel
    executor = MultiThreadedExecutor()
    executor.add_node(dialog_manager)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        dialog_manager.shutdown()
        executor.shutdown()
        dialog_manager.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()