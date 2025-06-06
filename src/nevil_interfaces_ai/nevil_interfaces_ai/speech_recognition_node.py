#!/usr/bin/env python3

import os
import uuid
import json
import threading
import queue
import numpy as np
import speech_recognition as sr
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import Bool, String
from sensor_msgs.msg import Audio
from nevil_interfaces_ai.msg import VoiceCommand, TextCommand, DialogState

class SpeechRecognitionNode(Node):
    """
    Speech recognition node for Nevil-picar v2.0.
    
    This node listens for audio input from a microphone and converts
    it to text using speech recognition. It supports both online and
    offline recognition modes.
    """
    
    def __init__(self):
        super().__init__('speech_recognition_node')
        
        # Declare parameters
        self.declare_parameter('use_online_recognition', True)
        self.declare_parameter('online_api', 'google')  # 'google', 'whisper', etc.
        self.declare_parameter('language', 'en-US')
        self.declare_parameter('energy_threshold', 300)
        self.declare_parameter('pause_threshold', 0.8)
        self.declare_parameter('dynamic_energy_threshold', True)
        self.declare_parameter('api_key', '')
        
        # Get parameters
        self.use_online = self.get_parameter('use_online_recognition').value
        self.online_api = self.get_parameter('online_api').value
        self.language = self.get_parameter('language').value
        self.energy_threshold = self.get_parameter('energy_threshold').value
        self.pause_threshold = self.get_parameter('pause_threshold').value
        self.dynamic_energy = self.get_parameter('dynamic_energy_threshold').value
        self.api_key = self.get_parameter('api_key').value
        
        # Create callback groups
        self.cb_group_subs = MutuallyExclusiveCallbackGroup()
        self.cb_group_pubs = MutuallyExclusiveCallbackGroup()
        
        # Create publishers
        self.voice_command_pub = self.create_publisher(
            VoiceCommand,
            '/nevil/voice_command',
            10
        )
        
        self.text_command_pub = self.create_publisher(
            TextCommand,
            '/nevil/text_command',
            10
        )
        
        self.raw_audio_pub = self.create_publisher(
            Audio,
            '/nevil/raw_audio',
            10
        )
        
        # Create subscribers
        self.listen_trigger_sub = self.create_subscription(
            Bool,
            '/nevil/listen_trigger',
            self.listen_trigger_callback,
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
        
        # Initialize speech recognition
        self.recognizer = sr.Recognizer()
        self.recognizer.energy_threshold = self.energy_threshold
        self.recognizer.pause_threshold = self.pause_threshold
        self.recognizer.dynamic_energy_threshold = self.dynamic_energy
        
        # Initialize state variables
        self.is_listening = False
        self.current_context_id = str(uuid.uuid4())
        self.current_dialog_state = 'idle'
        
        # Create audio processing queue and thread
        self.audio_queue = queue.Queue()
        self.stop_event = threading.Event()
        self.audio_thread = threading.Thread(target=self.audio_processing_thread)
        self.audio_thread.daemon = True
        self.audio_thread.start()
        
        self.get_logger().info('Speech recognition node initialized')
        
        # Start listening if auto-start is enabled
        self.start_listening()
    
    def listen_trigger_callback(self, msg):
        """Handle listen trigger messages."""
        if msg.data and not self.is_listening:
            self.start_listening()
        elif not msg.data and self.is_listening:
            self.stop_listening()
    
    def dialog_state_callback(self, msg):
        """Handle dialog state updates."""
        self.current_context_id = msg.context_id
        self.current_dialog_state = msg.state
        
        # Automatically start/stop listening based on dialog state
        if msg.state == 'listening' and not self.is_listening:
            self.start_listening()
        elif msg.state == 'speaking' and self.is_listening:
            self.stop_listening()
    
    def start_listening(self):
        """Start listening for speech."""
        if self.is_listening:
            return
        
        self.is_listening = True
        self.get_logger().info('Started listening for speech')
        
        # Start microphone in a separate thread to avoid blocking
        threading.Thread(target=self.listen_microphone).start()
    
    def stop_listening(self):
        """Stop listening for speech."""
        self.is_listening = False
        self.get_logger().info('Stopped listening for speech')
    
    def listen_microphone(self):
        """Listen to the microphone and add audio to the processing queue."""
        try:
            with sr.Microphone() as source:
                self.get_logger().info('Adjusting for ambient noise...')
                self.recognizer.adjust_for_ambient_noise(source, duration=1)
                
                self.get_logger().info('Listening for speech...')
                while self.is_listening:
                    try:
                        audio = self.recognizer.listen(source, timeout=10.0, phrase_time_limit=10.0)
                        self.audio_queue.put(audio)
                        self.get_logger().info('Audio captured, added to processing queue')
                    except sr.WaitTimeoutError:
                        self.get_logger().info('No speech detected, continuing to listen')
                    except Exception as e:
                        self.get_logger().error(f'Error capturing audio: {e}')
                        break
        except Exception as e:
            self.get_logger().error(f'Error initializing microphone: {e}')
            self.is_listening = False
    
    def audio_processing_thread(self):
        """Process audio from the queue in a separate thread."""
        while not self.stop_event.is_set():
            try:
                # Get audio from queue with timeout
                try:
                    audio = self.audio_queue.get(timeout=1.0)
                except queue.Empty:
                    continue
                
                # Process the audio
                self.process_audio(audio)
                
                # Mark task as done
                self.audio_queue.task_done()
                
            except Exception as e:
                self.get_logger().error(f'Error in audio processing thread: {e}')
    
    def process_audio(self, audio):
        """Process audio data and convert to text."""
        try:
            # Publish raw audio data if needed
            # self.publish_raw_audio(audio)
            
            # Recognize speech
            if self.use_online and self.online_api == 'google':
                text = self.recognizer.recognize_google(audio, language=self.language)
            elif self.use_online and self.online_api == 'whisper':
                text = self.recognizer.recognize_whisper(audio, language=self.language)
            else:
                # Fallback to offline recognition
                text = self.recognizer.recognize_sphinx(audio, language=self.language)
            
            self.get_logger().info(f'Recognized: {text}')
            
            # Create and publish voice command
            self.publish_voice_command(text, audio)
            
            # Also publish as text command for processing
            self.publish_text_command(text)
            
        except sr.UnknownValueError:
            self.get_logger().info('Speech not recognized')
        except sr.RequestError as e:
            self.get_logger().error(f'Error with speech recognition service: {e}')
            # Fallback to offline recognition if online fails
            if self.use_online:
                try:
                    text = self.recognizer.recognize_sphinx(audio, language=self.language)
                    self.get_logger().info(f'Fallback recognized: {text}')
                    self.publish_voice_command(text, audio)
                    self.publish_text_command(text)
                except Exception as e2:
                    self.get_logger().error(f'Fallback recognition also failed: {e2}')
        except Exception as e:
            self.get_logger().error(f'Error processing audio: {e}')
    
    def publish_raw_audio(self, audio):
        """Publish raw audio data."""
        try:
            # Convert audio data to the format expected by ROS
            audio_data = np.frombuffer(audio.get_raw_data(), dtype=np.int16)
            
            # Create Audio message
            msg = Audio()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'microphone'
            msg.data = audio_data.tobytes()
            msg.format = 'S16LE'  # 16-bit signed little-endian
            msg.sample_rate = 16000  # Typical sample rate
            msg.step = 2  # 2 bytes per sample (16-bit)
            msg.is_bigendian = False
            
            # Publish the message
            self.raw_audio_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing raw audio: {e}')
    
    def publish_voice_command(self, text, audio):
        """Publish recognized text as a voice command."""
        try:
            # Create command ID
            command_id = str(uuid.uuid4())
            
            # Create VoiceCommand message
            msg = VoiceCommand()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.recognized_text = text
            msg.confidence = 0.8  # This would be provided by the recognition engine
            msg.command_id = command_id
            msg.context_id = self.current_context_id
            
            # Optionally include audio data
            # self.publish_raw_audio(audio)
            
            # Publish the message
            self.voice_command_pub.publish(msg)
            self.get_logger().info(f'Published voice command: {text}')
            
        except Exception as e:
            self.get_logger().error(f'Error publishing voice command: {e}')
    
    def publish_text_command(self, text):
        """Publish recognized text as a text command."""
        try:
            # Create command ID
            command_id = str(uuid.uuid4())
            
            # Create TextCommand message
            msg = TextCommand()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.command_text = text
            msg.source = 'voice'
            msg.command_type = 'query'  # Default type, will be determined by processor
            msg.priority = 100  # Medium priority
            msg.command_id = command_id
            msg.context_id = self.current_context_id
            
            # Publish the message
            self.text_command_pub.publish(msg)
            self.get_logger().info(f'Published text command: {text}')
            
        except Exception as e:
            self.get_logger().error(f'Error publishing text command: {e}')
    
    def shutdown(self):
        """Clean up resources."""
        self.stop_event.set()
        self.stop_listening()
        if self.audio_thread.is_alive():
            self.audio_thread.join(timeout=1.0)


def main(args=None):
    rclpy.init(args=args)
    
    speech_recognition_node = SpeechRecognitionNode()
    
    # Use a MultiThreadedExecutor to enable processing multiple callbacks in parallel
    executor = MultiThreadedExecutor()
    executor.add_node(speech_recognition_node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        speech_recognition_node.shutdown()
        executor.shutdown()
        speech_recognition_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()