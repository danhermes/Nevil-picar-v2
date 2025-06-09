#!/usr/bin/env python3

"""
Direct script to run the speech interface components without relying on ROS2 launch.
This is a temporary solution until the package can be properly built and installed.
"""

import os
import sys
import subprocess
import time
import signal
import threading
import types

# Add the src directory to the Python path
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'src'))

# Create mock modules for nevil_interfaces_ai and nevil_interfaces
# This is a workaround for the import errors

# Create mock classes for message types
class MockHeader:
    def __init__(self):
        self.stamp = None

class Audio:
    def __init__(self):
        self.header = MockHeader()
        self.data = bytes()
        self.format = ""
        self.sample_rate = 0
        self.channels = 0
        self.step = 0
        self.is_bigendian = False

class TextCommand:
    def __init__(self):
        self.header = MockHeader()
        self.command_text = ""
        self.source = ""
        self.command_type = ""
        self.priority = 0
        self.command_id = ""
        self.context_id = ""

class TextResponse:
    def __init__(self):
        self.header = MockHeader()
        self.response_text = ""
        self.source = ""
        self.response_id = ""
        self.command_id = ""
        self.context_id = ""

class VoiceCommand:
    def __init__(self):
        self.header = MockHeader()
        self.recognized_text = ""
        self.confidence = 0.0
        self.command_id = ""
        self.context_id = ""
        self.speaker_id = ""

class VoiceResponse:
    def __init__(self):
        self.header = MockHeader()
        self.text_to_speak = ""
        self.voice_id = ""
        self.speaking_rate = 0.0
        self.pitch = 0.0
        self.volume = 0.0
        self.command_id = ""
        self.context_id = ""

class DialogState:
    def __init__(self):
        self.header = MockHeader()
        self.context_id = ""
        self.state = ""
        self.mode = ""
        self.dialog_history = ""
        self.turn_count = 0
        self.environment_context = ""

class SystemStatus:
    def __init__(self):
        self.header = MockHeader()
        self.mode = ""
        self.battery_level = 0.0
        self.system_ok = True

# Create mock modules
nevil_interfaces_ai_msg = types.ModuleType('nevil_interfaces_ai.msg')
nevil_interfaces_ai_msg.Audio = Audio
nevil_interfaces_ai_msg.TextCommand = TextCommand
nevil_interfaces_ai_msg.TextResponse = TextResponse
nevil_interfaces_ai_msg.VoiceCommand = VoiceCommand
nevil_interfaces_ai_msg.VoiceResponse = VoiceResponse
nevil_interfaces_ai_msg.DialogState = DialogState

# Create mock action classes
class ProcessDialogGoal:
    def __init__(self):
        self.context_id = ""
        self.initial_utterance = ""
        self.dialog_mode = ""
        self.timeout = 60.0

class ProcessDialogResult:
    def __init__(self):
        self.success = False
        self.message = ""
        self.final_state = ""
        self.dialog_summary = ""
        self.actions_taken = []

class ProcessDialogFeedback:
    def __init__(self):
        self.current_state = ""
        self.last_utterance = ""
        self.turn_count = 0
        self.elapsed_time = 0.0

class ProcessDialog:
    Goal = ProcessDialogGoal
    Result = ProcessDialogResult
    Feedback = ProcessDialogFeedback

# Create mock action module
nevil_interfaces_ai_action = types.ModuleType('nevil_interfaces_ai.action')
nevil_interfaces_ai_action.ProcessDialog = ProcessDialog

# Add to sys.modules
sys.modules['nevil_interfaces_ai.action'] = nevil_interfaces_ai_action

# Create mock service classes
class TranslateCommandRequest:
    def __init__(self):
        self.command_text = ""
        self.source = ""
        self.context_id = ""

class TranslateCommandResponse:
    def __init__(self):
        self.success = True
        self.translated_command = ""
        self.command_type = ""
        self.parameters = ""

class TranslateCommand:
    Request = TranslateCommandRequest
    Response = TranslateCommandResponse

# Create mock service module
nevil_interfaces_ai_srv = types.ModuleType('nevil_interfaces_ai.srv')
nevil_interfaces_ai_srv.TranslateCommand = TranslateCommand

# Add to sys.modules
sys.modules['nevil_interfaces_ai.srv'] = nevil_interfaces_ai_srv

nevil_interfaces_msg = types.ModuleType('nevil_interfaces.msg')
nevil_interfaces_msg.SystemStatus = SystemStatus

# Create mock ROS2 classes
class Node:
    def __init__(self, name):
        self.name = name
        self.publishers = {}
        self.subscriptions = {}
        self.clients = {}
        self.action_clients = {}
        
    def create_publisher(self, msg_type, topic, qos_profile):
        self.publishers[topic] = (msg_type, qos_profile)
        return Publisher(topic, msg_type)
    
    def create_subscription(self, msg_type, topic, callback, qos_profile, callback_group=None):
        self.subscriptions[topic] = (msg_type, callback, qos_profile)
        return Subscription(topic, msg_type, callback)
    
    def create_client(self, srv_type, srv_name, callback_group=None):
        self.clients[srv_name] = srv_type
        return Client(srv_name, srv_type)
    
    def get_clock(self):
        return Clock()
    
    def get_logger(self):
        return Logger(self.name)
    
    def declare_parameter(self, name, value):
        return Parameter(name, value)
    
    def get_parameter(self, name):
        return Parameter(name, None)
    
    def destroy_node(self):
        pass
    
    def destroy_subscription(self, subscription):
        pass
    
    def destroy_client(self, client):
        pass

class Publisher:
    def __init__(self, topic, msg_type):
        self.topic = topic
        self.msg_type = msg_type
    
    def publish(self, msg):
        print(f"Publishing to {self.topic}: {msg}")

class Subscription:
    def __init__(self, topic, msg_type, callback):
        self.topic = topic
        self.msg_type = msg_type
        self.callback = callback

class Client:
    def __init__(self, srv_name, srv_type):
        self.srv_name = srv_name
        self.srv_type = srv_type
    
    def wait_for_service(self, timeout_sec=1.0):
        return True
    
    def call_async(self, request):
        future = Future()
        response = self.srv_type.Response()
        future.set_result(response)
        return future

class ActionServer:
    def __init__(self, node, action_type, action_name, execute_callback=None, goal_callback=None, cancel_callback=None, callback_group=None):
        self.node = node
        self.action_type = action_type
        self.action_name = action_name
        self.execute_callback = execute_callback
        self.goal_callback = goal_callback
        self.cancel_callback = cancel_callback

class Future:
    def __init__(self):
        self._result = None
        self._done = False
    
    def set_result(self, result):
        self._result = result
        self._done = True
    
    def result(self):
        return self._result
    
    def done(self):
        return self._done

class Clock:
    def now(self):
        return self
    
    def to_msg(self):
        return None

class Logger:
    def __init__(self, name):
        self.name = name
    
    def info(self, msg):
        print(f"[INFO] [{self.name}]: {msg}")
    
    def warn(self, msg):
        print(f"[WARN] [{self.name}]: {msg}")
    
    def error(self, msg):
        print(f"[ERROR] [{self.name}]: {msg}")
    
    def debug(self, msg):
        print(f"[DEBUG] [{self.name}]: {msg}")

class Parameter:
    def __init__(self, name, value):
        self.name = name
        self.value = value

class MutuallyExclusiveCallbackGroup:
    def __init__(self):
        pass

class MultiThreadedExecutor:
    def __init__(self):
        self.nodes = []
    
    def add_node(self, node):
        self.nodes.append(node)
    
    def spin(self):
        while True:
            time.sleep(0.1)
    
    def shutdown(self):
        pass

class Bool:
    def __init__(self):
        self.data = False

class String:
    def __init__(self):
        self.data = ""

# Mock ROS2 initialization functions
def init(args=None):
    pass

def shutdown():
    pass

# Add the mock modules to sys.modules
sys.modules['nevil_interfaces_ai'] = types.ModuleType('nevil_interfaces_ai')
sys.modules['nevil_interfaces_ai.msg'] = nevil_interfaces_ai_msg
sys.modules['nevil_interfaces'] = types.ModuleType('nevil_interfaces')
sys.modules['nevil_interfaces.msg'] = nevil_interfaces_msg

# Add ROS2 mock modules
rclpy_module = types.ModuleType('rclpy')
rclpy_module.init = init
rclpy_module.shutdown = shutdown
rclpy_module.node = types.ModuleType('rclpy.node')
rclpy_module.node.Node = Node
rclpy_module.executors = types.ModuleType('rclpy.executors')
rclpy_module.executors.MultiThreadedExecutor = MultiThreadedExecutor
rclpy_module.callback_groups = types.ModuleType('rclpy.callback_groups')
rclpy_module.callback_groups.MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup
rclpy_module.action = types.ModuleType('rclpy.action')
rclpy_module.action.ActionServer = ActionServer

# Add missing action classes needed by dialog_manager_node.py
class CancelResponse:
    ACCEPT = 1
    REJECT = 2

class GoalResponse:
    ACCEPT = 1
    REJECT = 2

class ServerGoalHandle:
    def __init__(self, goal_id=None):
        self.goal_id = goal_id
        self._is_active = True
    
    def publish_feedback(self, feedback):
        print(f"Publishing feedback: {feedback}")
    
    def succeed(self):
        self._is_active = False
        print("Goal succeeded")
    
    def abort(self):
        self._is_active = False
        print("Goal aborted")
    
    @property
    def is_active(self):
        return self._is_active

rclpy_module.action.CancelResponse = CancelResponse
rclpy_module.action.GoalResponse = GoalResponse
rclpy_module.action.ServerGoalHandle = ServerGoalHandle

std_msgs_module = types.ModuleType('std_msgs.msg')
std_msgs_module.Bool = Bool
std_msgs_module.String = String

sys.modules['rclpy'] = rclpy_module
sys.modules['rclpy.node'] = rclpy_module.node
sys.modules['rclpy.executors'] = rclpy_module.executors
sys.modules['rclpy.callback_groups'] = rclpy_module.callback_groups
sys.modules['rclpy.action'] = rclpy_module.action

# Create rclpy.action.server module
rclpy_action_server_module = types.ModuleType('rclpy.action.server')
rclpy_action_server_module.ServerGoalHandle = ServerGoalHandle
sys.modules['rclpy.action.server'] = rclpy_action_server_module

sys.modules['std_msgs.msg'] = std_msgs_module

# Create a mock AudioHardwareInterface class
class AudioHardwareInterface:
    """
    Mock audio hardware interface for Nevil-picar v2.0.
    
    This class simulates the behavior of the real AudioHardwareInterface
    without requiring the actual hardware or dependencies.
    """
    
    def __init__(self, node=None):
        """
        Initialize the mock audio hardware interface.
        
        Args:
            node: ROS2 node for logging (optional)
        """
        self.node = node
        self.logger = node.get_logger() if node else Logger('audio_hardware_interface')
        
        # Initialize state variables
        self.simulation_mode = True
        self.language = 'en'
        self.volume_db = 3.0
        self.tts_voice = 'onyx'
        self.whisper_model = 'small'
        self.openai_api_key = os.environ.get('OPENAI_API_KEY', None)
        
        # Speech recognition parameters
        self.energy_threshold = 300
        self.pause_threshold = 0.5
        self.dynamic_energy_threshold = True
        
        self.logger.info('Mock audio hardware interface initialized in simulation mode')
    
    def get_microphone_list(self):
        """
        Get a list of available microphones (mock implementation).
        
        Returns:
            List of available microphone devices
        """
        self.logger.debug('Mock: Getting microphone list')
        return ["Simulated Microphone"]
    
    def get_speaker_list(self):
        """
        Get a list of available speakers (mock implementation).
        
        Returns:
            List of available speaker devices
        """
        self.logger.debug('Mock: Getting speaker list')
        return ["Simulated Speaker"]
    
    def set_microphone_device(self, device_index):
        """
        Set the microphone device (mock implementation).
        
        Args:
            device_index: Index of the microphone device to use
        """
        self.logger.debug(f'Mock: Setting microphone device to {device_index}')
    
    def set_speaker_voice(self, voice_id):
        """
        Set the speaker voice (mock implementation).
        
        Args:
            voice_id: ID of the voice to use
        """
        self.logger.debug(f'Mock: Setting speaker voice to {voice_id}')
        self.tts_voice = voice_id
    
    def set_speech_rate(self, rate):
        """
        Set the speech rate (mock implementation).
        
        Args:
            rate: Speech rate (words per minute)
        """
        self.logger.debug(f'Mock: Setting speech rate to {rate}')
    
    def set_speech_volume(self, volume):
        """
        Set the speech volume (mock implementation).
        
        Args:
            volume: Volume level (0.0 to 1.0)
        """
        self.logger.debug(f'Mock: Setting speech volume to {volume}')
        self.volume_db = volume * 10.0  # Convert to dB scale
    
    def listen_for_speech(self, timeout=10.0, phrase_time_limit=10.0, adjust_for_ambient_noise=True):
        """
        Listen for speech (mock implementation).
        
        Args:
            timeout: Maximum time to wait for speech (seconds)
            phrase_time_limit: Maximum time for a phrase (seconds)
            adjust_for_ambient_noise: Whether to adjust for ambient noise before listening
        
        Returns:
            Mock audio data
        """
        self.logger.debug(f'Mock: Listening for speech (timeout={timeout}s, phrase_time_limit={phrase_time_limit}s)')
        time.sleep(1.0)  # Simulate listening time
        
        # Return a mock audio object
        return MockAudio("This is simulated speech")
    
    def recognize_speech(self, audio, language=None, use_online=True, api='auto'):
        """
        Recognize speech (mock implementation).
        
        Args:
            audio: Audio data to recognize
            language: Language code (defaults to self.language if None)
            use_online: Whether to use online recognition
            api: API to use for online recognition
        
        Returns:
            Recognized text
        """
        if language is None:
            language = self.language
            
        self.logger.debug(f'Mock: Recognizing speech (language={language}, use_online={use_online}, api={api})')
        
        # If audio is our mock object, return its text
        if isinstance(audio, MockAudio):
            return audio.text
        
        # Otherwise return a default response
        return "This is simulated speech recognition"
    
    def speak_text(self, text, voice=None, wait=True):
        """
        Speak text (mock implementation).
        
        Args:
            text: Text to speak
            voice: Voice to use (optional)
            wait: Whether to wait for speech to complete
        """
        voice_str = voice if voice else self.tts_voice
        self.logger.debug(f'Mock: Speaking text with voice {voice_str}: "{text}"')
        if wait:
            # Simulate speaking time based on text length
            speak_time = len(text) * 0.05  # ~200 words per minute
            time.sleep(speak_time)
    
    def set_speech_recognition_parameters(self, energy_threshold=None, pause_threshold=None, dynamic_energy=None):
        """
        Set speech recognition parameters (mock implementation).
        
        Args:
            energy_threshold: Energy level threshold
            pause_threshold: Seconds of non-speaking audio
            dynamic_energy: Whether to dynamically adjust the energy threshold
        """
        if energy_threshold is not None:
            self.energy_threshold = energy_threshold
            self.logger.debug(f'Mock: Set energy threshold to {energy_threshold}')
        
        if pause_threshold is not None:
            self.pause_threshold = pause_threshold
            self.logger.debug(f'Mock: Set pause threshold to {pause_threshold}')
        
        if dynamic_energy is not None:
            self.dynamic_energy_threshold = dynamic_energy
            self.logger.debug(f'Mock: Set dynamic energy to {dynamic_energy}')
    
    def cleanup(self):
        """
        Clean up resources (mock implementation).
        """
        self.logger.info('Mock: Cleaning up audio hardware resources')

# Mock audio class to simulate speech recognition audio data
class MockAudio:
    def __init__(self, text="This is simulated speech"):
        self.text = text
        self._raw_data = b'mock audio data'
    
    def get_raw_data(self):
        return self._raw_data
    
    def get_wav_data(self):
        return self._raw_data

# Add the mock AudioHardwareInterface to the nevil_interfaces_ai module
sys.modules['nevil_interfaces_ai.audio_hardware_interface'] = types.ModuleType('nevil_interfaces_ai.audio_hardware_interface')
sys.modules['nevil_interfaces_ai.audio_hardware_interface'].AudioHardwareInterface = AudioHardwareInterface

# Define paths to the Python files
nevil_interfaces_ai_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                       'src/nevil_interfaces_ai/nevil_interfaces_ai')

speech_recognition_path = os.path.join(nevil_interfaces_ai_dir, 'speech_recognition_node.py')
speech_synthesis_path = os.path.join(nevil_interfaces_ai_dir, 'speech_synthesis_node.py')
dialog_manager_path = os.path.join(nevil_interfaces_ai_dir, 'dialog_manager_node.py')

# Verify files exist
print(f"Speech recognition path: {speech_recognition_path}, exists: {os.path.exists(speech_recognition_path)}")
print(f"Speech synthesis path: {speech_synthesis_path}, exists: {os.path.exists(speech_synthesis_path)}")
print(f"Dialog manager path: {dialog_manager_path}, exists: {os.path.exists(dialog_manager_path)}")

# Make sure the files are executable
for path in [speech_recognition_path, speech_synthesis_path, dialog_manager_path]:
    if os.path.exists(path):
        os.chmod(path, 0o755)

# Import the node classes directly
print("Importing node classes...")

# Add the nevil_interfaces_ai directory to the Python path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# Import the node classes
try:
    # First, try to import the speech recognition node
    if os.path.exists(speech_recognition_path):
        print("Importing speech recognition node...")
        # Use importlib to import the module
        import importlib.util
        spec = importlib.util.spec_from_file_location("speech_recognition_node", speech_recognition_path)
        speech_recognition_module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(speech_recognition_module)
        SpeechRecognitionNode = speech_recognition_module.SpeechRecognitionNode
        print("Speech recognition node imported successfully")
    else:
        print("Speech recognition node not found")
        SpeechRecognitionNode = None
    
    # Next, try to import the speech synthesis node
    if os.path.exists(speech_synthesis_path):
        print("Importing speech synthesis node...")
        spec = importlib.util.spec_from_file_location("speech_synthesis_node", speech_synthesis_path)
        speech_synthesis_module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(speech_synthesis_module)
        SpeechSynthesisNode = speech_synthesis_module.SpeechSynthesisNode
        print("Speech synthesis node imported successfully")
    else:
        print("Speech synthesis node not found")
        SpeechSynthesisNode = None
    
    # Finally, try to import the dialog manager node
    if os.path.exists(dialog_manager_path):
        print("Importing dialog manager node...")
        spec = importlib.util.spec_from_file_location("dialog_manager_node", dialog_manager_path)
        dialog_manager_module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(dialog_manager_module)
        DialogManagerNode = dialog_manager_module.DialogManager  # The class is named DialogManager, not DialogManagerNode
        print("Dialog manager node imported successfully")
    else:
        print("Dialog manager node not found")
        DialogManagerNode = None
except Exception as e:
    print(f"Error importing node classes: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)

# Create and run the nodes
nodes = []
executor = MultiThreadedExecutor()

# Create and add the speech recognition node
if SpeechRecognitionNode:
    print("Creating speech recognition node...")
    try:
        speech_recognition_node = SpeechRecognitionNode()
        nodes.append(speech_recognition_node)
        executor.add_node(speech_recognition_node)
        print("Speech recognition node created and added to executor")
    except Exception as e:
        print(f"Error creating speech recognition node: {e}")
        import traceback
        traceback.print_exc()

# Create and add the speech synthesis node
if SpeechSynthesisNode:
    print("Creating speech synthesis node...")
    try:
        speech_synthesis_node = SpeechSynthesisNode()
        nodes.append(speech_synthesis_node)
        executor.add_node(speech_synthesis_node)
        print("Speech synthesis node created and added to executor")
    except Exception as e:
        print(f"Error creating speech synthesis node: {e}")
        import traceback
        traceback.print_exc()

# Create and add the dialog manager node
if DialogManagerNode:
    print("Creating dialog manager node...")
    try:
        dialog_manager_node = DialogManagerNode()
        nodes.append(dialog_manager_node)
        executor.add_node(dialog_manager_node)
        print("Dialog manager node created and added to executor")
    except Exception as e:
        print(f"Error creating dialog manager node: {e}")
        import traceback
        traceback.print_exc()

# Handle Ctrl+C to gracefully shut down all nodes
def signal_handler(sig, frame):
    print("Shutting down all nodes...")
    for node in nodes:
        if hasattr(node, 'shutdown'):
            try:
                node.shutdown()
            except Exception as e:
                print(f"Error shutting down node: {e}")
    
    executor.shutdown()
    for node in nodes:
        try:
            node.destroy_node()
        except Exception as e:
            print(f"Error destroying node: {e}")
    
    rclpy.shutdown()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

# Run the executor
print("Starting executor...")
try:
    if nodes:
        executor.spin()
    else:
        print("No nodes were created. Exiting.")
except KeyboardInterrupt:
    pass
finally:
    signal_handler(signal.SIGINT, None)