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
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# Define a local get_env_var function
def get_env_var(name, default=None):
    """
    Get an environment variable, with fallback to a default value.
    
    Args:
        name: Name of the environment variable
        default: Default value if the environment variable is not set
        
    Returns:
        The value of the environment variable, or the default value
    """
    return os.environ.get(name, default)

from std_msgs.msg import Bool, String
from nevil_interfaces_ai_msgs.msg import Audio, VoiceCommand, TextCommand, DialogState

# Import the audio record interface
try:
    from nevil_interfaces_ai.nevil_audio_record import NevilAudioRecord
except ImportError:
    # Try relative import if package import fails
    from nevil_interfaces_ai.nevil_audio_record import NevilAudioRecord

class SpeechRecognitionNode(Node):
    """
    Speech Recog for Nevil-picar v2.0.
    
    This node listens for audio input from a microphone and converts
    it to text using speech recognition. It supports both online and
    offline recognition modes.
    """
    
    def __init__(self):
        super().__init__('speech_recognition_node')
        
        # Add debug logging for initialization
        self.get_logger().warning('🔊 Speech Recog: Initializing...')
        print("🔊 Speech Recog: Initializing...")
        
        # Declare parameters with defaults from environment variables
        self.declare_parameter('use_online_recognition', True)
        self.declare_parameter('online_api', get_env_var('SPEECH_RECOGNITION_API', 'auto'))  # 'openai', 'whisper', 'auto'
        self.declare_parameter('language', get_env_var('SPEECH_RECOGNITION_LANGUAGE', 'en-US'))
        
        
        # Get parameters
        self.use_online = self.get_parameter('use_online_recognition').value
        self.online_api = self.get_parameter('online_api').value
        self.language = self.get_parameter('language').value
        
        
        # Note: OpenAI API key is not needed for Whisper (offline speech-to-text)
        # but may be needed for other OpenAI services like GPT models
        self.api_key = get_env_var('OPENAI_API_KEY', '')
        if self.api_key:
            self.get_logger().info('OpenAI API key loaded from environment (for language processing)')
            # Set it in the environment for potential OpenAI API calls
            os.environ["OPENAI_API_KEY"] = self.api_key
        
        # Create callback groups
        self.cb_group_subs = MutuallyExclusiveCallbackGroup()
        self.cb_group_pubs = MutuallyExclusiveCallbackGroup()

        # QoS profile for speech messages
        self.sp_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )
        
        # # Create publishers
        # self.voice_command_pub = self.create_publisher(
        #     VoiceCommand,
        #     '/nevil/voice_command',
        #     qos_profile=self.sp_qos
        # )
        
        self.text_command_pub = self.create_publisher(
            TextCommand,
            '/nevil/text_command',
            qos_profile=self.sp_qos
        )
        
        # self.raw_audio_pub = self.create_publisher(
        #     Audio,
        #     '/nevil/raw_audio',
        #     qos_profile=self.sp_qos
        # )
        
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
        
        # Subscribe to speaking status for immediate pause/resume
        self.speaking_status_sub = self.create_subscription(
            Bool,
            '/nevil/speaking_status',
            self.speaking_status_callback,
            qos_profile=self.sp_qos,
            callback_group=self.cb_group_subs
        )
        
        # Initialize audio hardware interface
        self.get_logger().warning('🔊 Speech Recog: Initializing audio RECORDINGhardware interface...')
        print("🔊 Speech Recog: Initializing audio RECORDING hardware interface...")
        self.audio_hw = NevilAudioRecord(self)
        
        # Check if audio hardware initialized properly
        if self.audio_hw.simulation_mode:
            self.get_logger().warning('🔊 Speech Recog: Audio hardware in simulation mode - microphone may not be available')
            print("🔊 Speech Recog: Audio hardware in simulation mode - microphone may not be available")
        elif not self.audio_hw.microphone:
            self.get_logger().error('🔊 Speech Recog: Microphone not available - speech recognition disabled')
            print("🔊 Speech Recog: Microphone not available - speech recognition disabled")
        else:
            self.get_logger().warning('🔊 Speech Recog: Audio hardware initialized successfully')
            print("🔊 Speech Recog: Audio hardware initialized successfully")
        
        
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
        
        self.get_logger().warning('🔊 Speech Recog: Speech Recog initialized')
        print("🔊 Speech Recog: Speech Recog initialized")
        
        # Start listening if auto-start is enabled
        self.get_logger().warning('🔊 Speech Recog: Starting listening...')
        print("🔊 Speech Recog: Starting listening...")
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
    
    def speaking_status_callback(self, msg):
        """Handle speaking status updates for immediate pause/resume."""
        is_speaking = msg.data
        
        if is_speaking and self.is_listening:
            # Immediately pause listening when speech synthesis starts
            self.stop_listening()
            self.get_logger().info('Paused speech recognition - Nevil is speaking')
        elif not is_speaking and not self.is_listening:
            # Resume listening when speech synthesis stops (no dialog manager dependency)
            self.start_listening()
            self.get_logger().info('Resumed speech recognition - Nevil finished speaking')
    
    def start_listening(self):
        """Start listening for speech."""
        if self.is_listening:
            self.get_logger().warning('🔊 Speech Recog: Already listening, skipping start_listening')
            print("🔊 Speech Recog: Already listening, skipping start_listening")
            return
        
        self.is_listening = True
        self.get_logger().warning('🔊 Speech Recog: Started listening for speech')
        print("🔊 Speech Recog: Started listening for speech")
        
        # Check microphone availability before starting thread
        if not self.audio_hw.microphone:
            self.get_logger().error('🔊 Speech Recog: Cannot start listening - microphone not available')
            print("🔊 Speech Recog: Cannot start listening - microphone not available")
            self.is_listening = False
            return
        
        # Start microphone in a separate thread to avoid blocking
        self.get_logger().warning('🔊 Speech Recog: Starting microphone thread...')
        print("🔊 Speech Recog: Starting microphone thread...")
        threading.Thread(target=self.listen_microphone).start()
    
    def stop_listening(self):
        """Stop listening for speech."""
        self.is_listening = False
        self.get_logger().info('Stopped listening for speech')
    
    def listen_microphone(self):
        """Listen to the microphone and add audio to the processing queue."""
        self.get_logger().warning('🔊 SPEECH RECOGNITION: Listening for speech...')
        print("🔊 SPEECH RECOGNITION: Listening for speech...")
        
        while self.is_listening:
            try:
                # Check if microphone is available before attempting to listen
                if not self.audio_hw.microphone:
                    self.get_logger().error('Microphone is not available - stopping speech recognition')
                    self.get_logger().error('Speech recognition disabled due to microphone failure')
                    break
                
                # Use the audio hardware interface to listen for speech
                # Use the new adjust_for_ambient_noise parameter
                audio = self.audio_hw.listen_for_speech(
                    timeout=10.0,
                    phrase_time_limit=10.0,
                    adjust_for_ambient_noise=True
                )
                
                if audio:
                    self.audio_queue.put(audio)
                    self.get_logger().info('Audio captured, added to processing queue')
                else:
                    # Check if microphone became corrupted during listening
                    if not self.audio_hw.microphone:
                        self.get_logger().error('Microphone became corrupted during listening - stopping speech recognition')
                        break
                    self.get_logger().info('No speech detected, continuing to listen')
            except Exception as e:
                self.get_logger().error(f'Error capturing audio: {e}')
                break
    
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
        if audio is None:
            return
        
        # Don't process audio if we're not supposed to be listening
        if not self.is_listening:
            self.get_logger().debug('Skipping audio processing - not listening')
            return
            
        try:
            # Use the audio hardware interface to recognize speech
            # Use the updated API with auto selection and language format
            text = self.audio_hw.recognize_speech(
                audio,
                language=self.language,  # The interface will handle language code conversion
                use_online=self.use_online,
                api='auto' if self.online_api == 'auto' else self.online_api
            )
            
            if text:
                self.get_logger().warning(f'🔊 SPEECH RECOGNITION: Recognized: {text}')
                print(f"🔊 SPEECH RECOGNITION: Recognized: {text}")
                
                # Create and publish voice command
                self.publish_voice_command(text, audio)
                
                # Also publish as text command for processing
                self.publish_text_command(text)
            else:
                self.get_logger().info('Speech not recognized')
                
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
            # self.voice_command_pub.publish(msg)
            self.get_logger().info(f'Voice command disabled (no dialog manager): {text}')
            
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
        
        # Clean up the audio hardware interface
        self.audio_hw.cleanup()


def main(args=None):
    import signal
    
    rclpy.init(args=args)
    
    speech_recognition_node = SpeechRecognitionNode()
    
    # Use a MultiThreadedExecutor to enable processing multiple callbacks in parallel
    executor = MultiThreadedExecutor()
    executor.add_node(speech_recognition_node)
    
    # Set up signal handlers for proper cleanup
    def signal_handler(signum, frame):
        speech_recognition_node.get_logger().info(f'Received signal {signum}, shutting down...')
        speech_recognition_node.shutdown()
        executor.shutdown()
        speech_recognition_node.destroy_node()
        rclpy.shutdown()
        exit(0)
    
    signal.signal(signal.SIGTERM, signal_handler)
    signal.signal(signal.SIGINT, signal_handler)
    
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