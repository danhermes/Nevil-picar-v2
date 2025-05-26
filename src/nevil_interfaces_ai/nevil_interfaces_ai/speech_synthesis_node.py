#!/usr/bin/env python3

import os
import uuid
import json
import threading
import queue
import numpy as np
import pyttsx3
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import Bool, String
from sensor_msgs.msg import Audio
from nevil_interfaces_ai.msg import VoiceResponse, TextResponse, DialogState

class SpeechSynthesisNode(Node):
    """
    Speech synthesis node for Nevil-picar v2.0.
    
    This node converts text to speech using either online or offline
    text-to-speech engines. It supports both local pyttsx3 and
    online services like Google Text-to-Speech.
    """
    
    def __init__(self):
        super().__init__('speech_synthesis_node')
        
        # Declare parameters
        self.declare_parameter('use_online_tts', False)
        self.declare_parameter('online_service', 'google')  # 'google', 'azure', etc.
        self.declare_parameter('voice_id', '')
        self.declare_parameter('speaking_rate', 1.0)
        self.declare_parameter('pitch', 1.0)
        self.declare_parameter('volume', 1.0)
        self.declare_parameter('api_key', '')
        
        # Get parameters
        self.use_online = self.get_parameter('use_online_tts').value
        self.online_service = self.get_parameter('online_service').value
        self.voice_id = self.get_parameter('voice_id').value
        self.speaking_rate = self.get_parameter('speaking_rate').value
        self.pitch = self.get_parameter('pitch').value
        self.volume = self.get_parameter('volume').value
        self.api_key = self.get_parameter('api_key').value
        
        # Create callback groups
        self.cb_group_subs = MutuallyExclusiveCallbackGroup()
        self.cb_group_pubs = MutuallyExclusiveCallbackGroup()
        
        # Create publishers
        self.speaking_status_pub = self.create_publisher(
            Bool,
            '/nevil/speaking_status',
            10
        )
        
        self.audio_pub = self.create_publisher(
            Audio,
            '/nevil/synthesized_audio',
            10
        )
        
        # Create subscribers
        self.voice_response_sub = self.create_subscription(
            VoiceResponse,
            '/nevil/voice_response',
            self.voice_response_callback,
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
        
        self.dialog_state_sub = self.create_subscription(
            DialogState,
            '/nevil/dialog_state',
            self.dialog_state_callback,
            10,
            callback_group=self.cb_group_subs
        )
        
        # Initialize TTS engine
        self.init_tts_engine()
        
        # Initialize state variables
        self.is_speaking = False
        self.current_context_id = ''
        self.current_dialog_state = 'idle'
        
        # Create speech processing queue and thread
        self.speech_queue = queue.Queue()
        self.stop_event = threading.Event()
        self.speech_thread = threading.Thread(target=self.speech_processing_thread)
        self.speech_thread.daemon = True
        self.speech_thread.start()
        
        self.get_logger().info('Speech synthesis node initialized')
    
    def init_tts_engine(self):
        """Initialize the text-to-speech engine."""
        try:
            if not self.use_online:
                # Initialize offline TTS engine (pyttsx3)
                self.engine = pyttsx3.init()
                
                # Configure voice properties
                self.engine.setProperty('rate', int(self.speaking_rate * 200))  # Default rate is 200
                self.engine.setProperty('volume', self.volume)
                
                # Set voice if specified
                if self.voice_id:
                    voices = self.engine.getProperty('voices')
                    for voice in voices:
                        if self.voice_id in voice.id:
                            self.engine.setProperty('voice', voice.id)
                            break
                
                self.get_logger().info('Initialized offline TTS engine (pyttsx3)')
            else:
                # Online TTS would be initialized here
                # This would typically involve setting up API clients
                self.get_logger().info(f'Initialized online TTS engine ({self.online_service})')
        
        except Exception as e:
            self.get_logger().error(f'Error initializing TTS engine: {e}')
            # Fallback to offline TTS if online fails
            if self.use_online:
                self.use_online = False
                self.init_tts_engine()
    
    def voice_response_callback(self, msg):
        """Handle voice response messages."""
        self.get_logger().info(f'Received voice response: {msg.text_to_speak}')
        
        # Add to speech queue
        self.speech_queue.put({
            'text': msg.text_to_speak,
            'voice_id': msg.voice_id if msg.voice_id else self.voice_id,
            'speaking_rate': msg.speaking_rate if msg.speaking_rate > 0.0 else self.speaking_rate,
            'pitch': msg.pitch if msg.pitch > 0.0 else self.pitch,
            'volume': msg.volume if msg.volume > 0.0 else self.volume,
            'command_id': msg.command_id,
            'context_id': msg.context_id
        })
    
    def text_response_callback(self, msg):
        """Handle text response messages and convert to speech."""
        self.get_logger().info(f'Received text response: {msg.response_text}')
        
        # Add to speech queue
        self.speech_queue.put({
            'text': msg.response_text,
            'voice_id': self.voice_id,
            'speaking_rate': self.speaking_rate,
            'pitch': self.pitch,
            'volume': self.volume,
            'command_id': msg.command_id,
            'context_id': msg.context_id
        })
    
    def dialog_state_callback(self, msg):
        """Handle dialog state updates."""
        self.current_context_id = msg.context_id
        self.current_dialog_state = msg.state
    
    def speech_processing_thread(self):
        """Process speech from the queue in a separate thread."""
        while not self.stop_event.is_set():
            try:
                # Get speech from queue with timeout
                try:
                    speech_data = self.speech_queue.get(timeout=1.0)
                except queue.Empty:
                    continue
                
                # Process the speech
                self.synthesize_speech(speech_data)
                
                # Mark task as done
                self.speech_queue.task_done()
                
            except Exception as e:
                self.get_logger().error(f'Error in speech processing thread: {e}')
    
    def synthesize_speech(self, speech_data):
        """Synthesize speech from text."""
        try:
            # Extract speech parameters
            text = speech_data['text']
            voice_id = speech_data['voice_id']
            speaking_rate = speech_data['speaking_rate']
            pitch = speech_data['pitch']
            volume = speech_data['volume']
            
            # Update speaking status
            self.is_speaking = True
            self.publish_speaking_status(True)
            
            if not self.use_online:
                # Use offline TTS (pyttsx3)
                
                # Update engine properties if they've changed
                self.engine.setProperty('rate', int(speaking_rate * 200))
                self.engine.setProperty('volume', volume)
                
                if voice_id:
                    voices = self.engine.getProperty('voices')
                    for voice in voices:
                        if voice_id in voice.id:
                            self.engine.setProperty('voice', voice.id)
                            break
                
                # Speak the text
                self.engine.say(text)
                self.engine.runAndWait()
            else:
                # Online TTS would be implemented here
                # This would typically involve API calls and audio playback
                self.get_logger().info(f'Would use online TTS for: {text}')
                
                # Simulate speech duration for now
                import time
                time.sleep(len(text) * 0.07)  # Rough estimate of speech duration
            
            # Update speaking status
            self.is_speaking = False
            self.publish_speaking_status(False)
            
            self.get_logger().info(f'Finished speaking: {text}')
            
        except Exception as e:
            self.get_logger().error(f'Error synthesizing speech: {e}')
            self.is_speaking = False
            self.publish_speaking_status(False)
    
    def publish_speaking_status(self, is_speaking):
        """Publish speaking status."""
        msg = Bool()
        msg.data = is_speaking
        self.speaking_status_pub.publish(msg)
    
    def shutdown(self):
        """Clean up resources."""
        self.stop_event.set()
        if self.speech_thread.is_alive():
            self.speech_thread.join(timeout=1.0)
        
        # Clean up TTS engine
        if hasattr(self, 'engine') and not self.use_online:
            try:
                self.engine.stop()
            except:
                pass


def main(args=None):
    rclpy.init(args=args)
    
    speech_synthesis_node = SpeechSynthesisNode()
    
    # Use a MultiThreadedExecutor to enable processing multiple callbacks in parallel
    executor = MultiThreadedExecutor()
    executor.add_node(speech_synthesis_node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        speech_synthesis_node.shutdown()
        executor.shutdown()
        speech_synthesis_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()