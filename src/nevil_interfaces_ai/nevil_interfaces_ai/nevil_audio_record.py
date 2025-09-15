#!/usr/bin/env python3

"""
Nevil Audio Record Interface for Nevil-picar v2.0

This module provides a thread-safe hardware interface for audio recording
on the Nevil-picar v2.0 system, interfacing with the microphone hardware.
Based on the original Nevil v1.0 implementation.

Extracted from AudioHardwareInterface to separate record functionality.
"""

import os
import time
import threading
import subprocess
import speech_recognition as sr
from rclpy.logging import get_logger

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

class NevilAudioRecord:
    """
    Audio recording interface for Nevil-picar v2.0.
    
    This class provides a thread-safe interface to the microphone hardware,
    with proper mutex handling for real-time performance.
    Based on the original Nevil v1.0 implementation.
    
    SINGLETON PATTERN: Only one instance can exist to manage the single physical microphone.
    """
    
    # Singleton instance
    _instance = None
    _lock = threading.Lock()
    
    # Default audio parameters
    DEFAULT_SAMPLE_RATE = 44100
    DEFAULT_CHANNELS = 1
    DEFAULT_CHUNK_SIZE = 1024
    DEFAULT_LANGUAGE = "en-US"
    # DEFAULT_ENERGY_THRESHOLD = 300
    # DEFAULT_PAUSE_THRESHOLD = 0.8
    # DEFAULT_DYNAMIC_ENERGY = True

    MICROPHONE_DEVICE_INDEX = 1  # USB PnP Sound Device: Audio (hw:3,0) - PyAudio index 1
    
    def __new__(cls, node=None):
        """Singleton pattern implementation."""
        if cls._instance is None:
            with cls._lock:
                if cls._instance is None:
                    cls._instance = super(NevilAudioRecord, cls).__new__(cls)
        return cls._instance
    
    def __init__(self, node=None):
        """
        Initialize the Audio Record Interface.
        
        Args:
            node: ROS2 node for logging (optional)
        """
        # Only initialize once for singleton
        if hasattr(self, '_initialized'):
            return
            
        self.node = node
        self.logger = get_logger('nevil_audio_record') if node is None else node.get_logger()
        
        self.logger.warning('🔊 Audio Record: RECORD DEVICES: arecord -l...')
        print("🔊 Audio Record: RECORD DEVICES: arecord -l...")
        
        # Display and log available recording devices
        try:
            result = subprocess.run(['arecord', '-l'], capture_output=True, text=True)
            self.logger.info(f'Available recording devices:\n{result.stdout}')
            print(f'🔊 Available recording devices:\n{result.stdout}')
            if result.stderr:
                self.logger.warning(f'arecord -l stderr: {result.stderr}')
                print(f'🔊 arecord -l stderr: {result.stderr}')
            
            # Also try arecord --list-devices for more detailed info
            result2 = subprocess.run(['arecord', '--list-devices'], capture_output=True, text=True)
            self.logger.info(f'Detailed recording devices:\n{result2.stdout}')
            print(f'🔊 Detailed recording devices:\n{result2.stdout}')
            if result2.stderr:
                self.logger.warning(f'arecord --list-devices stderr: {result2.stderr}')
                print(f'🔊 arecord --list-devices stderr: {result2.stderr}')
                
        except Exception as e:
            self.logger.error(f'Failed to get recording devices: {e}')
            print(f'🔊 Failed to get recording devices: {e}')
        
        # Initialize logging
        self.logger.warning('🔊 Audio Record: Initializing SINGLETON...')
        print("🔊 Audio Record: Initializing SINGLETON...")
        
        # Thread safety
        self.hardware_mutex = threading.Lock()
        
        # Initialize state
        self.simulation_mode = False
        
        # Configure default audio parameters
        self.sample_rate = self.DEFAULT_SAMPLE_RATE
        self.channels = self.DEFAULT_CHANNELS
        self.chunk_size = self.DEFAULT_CHUNK_SIZE
        self.language = get_env_var('SPEECH_RECOGNITION_LANGUAGE', self.DEFAULT_LANGUAGE)
        
        # Check for OpenAI API key
        self.openai_api_key = get_env_var('OPENAI_API_KEY', None)
        if self.openai_api_key:
            self.logger.info('OpenAI API key loaded for speech recognition')
        else:
            self.logger.warning('No OpenAI API key found - OpenAI recognition will not be available')
        
        # Initialize hardware components
        self.recognizer = None
        self.microphone = None
        
        # Check if audio libraries are available
        try:
            import speech_recognition as sr
            AUDIO_LIBS_AVAILABLE = True
        except ImportError:
            AUDIO_LIBS_AVAILABLE = False
            self.logger.warning('Speech recognition library not available')
        
        if not AUDIO_LIBS_AVAILABLE:
            self.logger.warning('🔊 Audio Record: Running in simulation mode')
            print("🔊 Audio Record: Running in simulation mode")
            self.recognizer = None
            self.microphone = None
            self.simulation_mode = True
        else:
            try:
                # Initialize speech recognition with environment variables
                self.recognizer = sr.Recognizer()
                self.logger.warning("🔊 Audio Record: [Initializing Audio]sr.Recognizer()")
                print("🔊 Audio Record: [Initializing Audio]sr.Recognizer()")

                # Set the 5 core speech_recognition library parameters
                self.recognizer.energy_threshold = 400         # Audio energy level for speech detection (50-4000 range) - HIGHER = less sensitive to ambient noise
                self.recognizer.pause_threshold = 1.2           # Seconds of silence to mark phrase end - LONGER to avoid ambient noise
                self.recognizer.phrase_threshold = 0.8          # Minimum speaking duration before considering phrase - LONGER to filter noise
                self.recognizer.non_speaking_duration = 0.3     # Non-speaking audio to keep on both sides
                self.recognizer.dynamic_energy_threshold = False # Disable dynamic adjustment to prevent drift and hallucinations

                # Initialize microphone
                self.logger.warning("🔊 Audio Record: [Initializing Audio]sr.Microphone")
                print("🔊 Audio Record: [Initializing Audio]sr.Microphone")
                
                # Try to use the default microphone with fallback sample rates
                sample_rates_to_try = [44100, 48000, 16000, 22050, 8000]
                microphone_initialized = False
                
                for sample_rate in sample_rates_to_try:
                    try:
                        self.logger.warning(f'🔊 Audio Record: Attempting to create sr.Microphone with sample rate {sample_rate}...')
                        print(f"🔊 Audio Record: Attempting to create sr.Microphone with sample rate {sample_rate}...")
                        # Use default device if MICROPHONE_DEVICE_INDEX is None
                        if self.MICROPHONE_DEVICE_INDEX is None:
                            self.microphone = sr.Microphone(chunk_size=self.chunk_size, sample_rate=sample_rate)
                        else:
                            self.microphone = sr.Microphone(device_index=self.MICROPHONE_DEVICE_INDEX,
                                  chunk_size=self.chunk_size, sample_rate=sample_rate)
                        self.sample_rate = sample_rate  # Update the sample rate to what worked
                        self.logger.warning(f'🔊 Audio Record: Microphone initialized successfully with sample rate {sample_rate}')
                        print(f"🔊 Audio Record: Microphone initialized successfully with sample rate {sample_rate}")
                        self.logger.warning(f'🔊 Audio Record: Microphone object: {self.microphone}')
                        print(f"🔊 Audio Record: Microphone object: {self.microphone}")
                        self.logger.warning(f'🔊 Audio Record: Microphone type: {type(self.microphone)}')
                        print(f"🔊 Audio Record: Microphone type: {type(self.microphone)}")
                        microphone_initialized = True
                        break
                    except Exception as e:
                        self.logger.warning(f'🔊 Audio Record: Failed to initialize microphone with sample rate {sample_rate}: {e}')
                        print(f"🔊 Audio Record: Failed to initialize microphone with sample rate {sample_rate}: {e}")
                        continue
                
                if not microphone_initialized:
                    self.logger.error('🔊 Audio Record: Failed to initialize microphone with any sample rate')
                    print("🔊 Audio Record: Failed to initialize microphone with any sample rate")
                    self.logger.error('🔊 Audio Record: Setting microphone to None due to microphone initialization failure')
                    print("🔊 Audio Record: Setting microphone to None due to microphone initialization failure")
                    self.microphone = None
                
                self.logger.warning('🔊 Audio Record: Audio record interface initialized')
                print("🔊 Audio Record: Audio record interface initialized")
                
            except Exception as e:
                self.logger.error(f'Failed to initialize audio record hardware: {e}')
                print(f"Failed to initialize audio record hardware: {e}")
                
                # Create mock audio interfaces for simulation if hardware initialization fails
                self.recognizer = None
                self.microphone = None
                self.simulation_mode = True
        
        # Mark as initialized for singleton pattern
        self._initialized = True

    @classmethod
    def get_instance(cls, node=None):
        """Get the singleton instance of NevilAudioRecord."""
        return cls(node)


    def listen_for_speech(self, timeout=10.0, phrase_time_limit=10.0, adjust_for_ambient_noise=True):
        """
        Listen for speech with proper mutex handling.
        
        Args:
            timeout: Maximum time to wait for speech (seconds)
            phrase_time_limit: Maximum time for a phrase (seconds)
            adjust_for_ambient_noise: Whether to adjust for ambient noise before listening
        
        Returns:
            Audio data from the microphone
        """
        if self.simulation_mode or not self.microphone:
            # In simulation mode, return mock audio data
            self.logger.debug('Simulation: Listening for speech')
            time.sleep(1.0)  # Simulate listening time
            return None
        
        # In hardware mode, listen for speech with mutex protection
        try:
            with self.hardware_mutex:
                # Double-check microphone is available before using it
                if not self.microphone:
                    self.logger.error('Microphone is None - singleton should maintain persistent connection')
                    return None
                
                # Additional safety check - ensure microphone is not None
                if self.microphone is None:
                    self.logger.error('Microphone is None after mutex check - cannot listen for speech')
                    self.logger.error('Microphone object became None during operation - possible corruption')
                    return None
                
                try:
                    with self.microphone as source:
                        self.logger.debug('Listening for speech...')
                        
                        # # Set energy threshold based on environment
                        # if self.recognizer.energy_threshold < self.DEFAULT_ENERGY_THRESHOLD:
                        #     self.recognizer.energy_threshold = self.DEFAULT_ENERGY_THRESHOLD
                        
                        if adjust_for_ambient_noise:
                            try:
                                self.logger.debug('Adjusting for ambient noise...')
                                self.recognizer.adjust_for_ambient_noise(source, duration=0.5)
                                self.logger.debug('adjust_for_ambient_noise: done')
                            except Exception as e:
                                self.logger.warning(f'Failed to adjust for ambient noise: {e}')
                                # Continue without ambient noise adjustment rather than failing completely
                                self.logger.info('Continuing without ambient noise adjustment')
                        
                        audio = self.recognizer.listen(source, timeout=timeout, phrase_time_limit=phrase_time_limit)
                        self.logger.debug('Speech captured')
                        return audio
                except AttributeError as e:
                    if "'NoneType' object has no attribute 'close'" in str(e):
                        self.logger.error('Microphone context manager failed - microphone corrupted')
                        self.logger.error('Speech recognition disabled due to microphone failure')
                        self.microphone = None
                        return None
                    else:
                        raise e
        except sr.WaitTimeoutError:
            self.logger.debug('No speech detected within timeout')
            return None
        except Exception as e:
            self.logger.error(f'Error listening for speech: {e}')
            return None

    def recognize_speech(self, audio, language=None, use_online=True, api='auto'):
        """
        Recognize speech with proper mutex handling.
        
        Args:
            audio: Audio data to recognize
            language: Language code (defaults to self.language if None)
            use_online: Whether to use online recognition
            api: API to use for online recognition ('openai', 'whisper-local', 'google', 'auto', etc.)
        
        Returns:
            Recognized text
        """
        if self.simulation_mode or audio is None:
            # In simulation mode, return mock text
            self.logger.debug('Simulation: Recognizing speech')
            return "This is simulated speech recognition"
        
        # Use default language if not specified
        if language is None:
            language = self.language
        
        try:
            with self.hardware_mutex:
                if not self.recognizer:
                    self.logger.error('Speech recognizer not available')
                    return None
                
                # Try different recognition methods
                recognition_methods = []
                
                if use_online:
                    if api in ['auto', 'openai'] and self.openai_api_key:
                        recognition_methods.append(('OpenAI Whisper', lambda: self.recognizer.recognize_openai(audio)))
                    # Google recognition removed - we will never use Google
                
                # Try each method until one succeeds
                for method_name, method_func in recognition_methods:
                    try:
                        self.logger.debug(f'Trying {method_name} recognition...')
                        text = method_func()
                        if text and text.strip():
                            self.logger.debug(f'{method_name} recognition successful: {text}')
                            return text.strip()
                    except Exception as e:
                        self.logger.warning(f'{method_name} recognition failed: {e}')
                        continue
                
                self.logger.error('All speech recognition methods failed')
                return ""
                
        except Exception as e:
            self.logger.error(f'Error recognizing speech: {e}')
            return ""

    def cleanup(self):
        """Clean up audio record resources."""
        try:
            with self.hardware_mutex:
                # Clean up microphone device (critical for USB audio device release)
                if self.microphone:
                    try:
                        # Force close the microphone device to release hardware
                        if hasattr(self.microphone, '__del__'):
                            self.microphone.__del__()
                        self.microphone = None
                        self.logger.info('Microphone device released')
                    except Exception as e:
                        self.logger.error(f'Failed to release microphone: {e}')
                
                # Clean up recognizer
                if self.recognizer:
                    self.recognizer = None
                    self.logger.info('Speech recognizer cleaned up')
                    
        except Exception as e:
            self.logger.error(f'Error during audio record cleanup: {e}')

    def _recover_microphone(self):
        """Attempt to recover the microphone after it becomes corrupted."""
        try:
            self.logger.warning('Attempting microphone recovery...')
            
            # Clean up the corrupted microphone
            if self.microphone:
                try:
                    if hasattr(self.microphone, '__del__'):
                        self.microphone.__del__()
                except Exception as e:
                    self.logger.debug(f'Error during microphone cleanup: {e}')
                self.microphone = None
            
            # Wait a moment for hardware to reset
            time.sleep(0.5)
            
            # Try to reinitialize the microphone
            sample_rates_to_try = [44100, 48000, 16000, 22050, 8000]
            microphone_initialized = False
            
            for sample_rate in sample_rates_to_try:
                try:
                    self.logger.warning(f'Recovery: Attempting to create sr.Microphone with sample rate {sample_rate}...')
                    # Use default device if MICROPHONE_DEVICE_INDEX is None
                    if self.MICROPHONE_DEVICE_INDEX is None:
                        self.microphone = sr.Microphone(chunk_size=self.chunk_size, sample_rate=sample_rate)
                    else:
                        self.microphone = sr.Microphone(device_index=self.MICROPHONE_DEVICE_INDEX, chunk_size=self.chunk_size, sample_rate=sample_rate)
                    self.sample_rate = sample_rate
                    self.logger.warning(f'Recovery: Microphone recovered successfully with sample rate {sample_rate}')
                    microphone_initialized = True
                    break
                except Exception as e:
                    self.logger.warning(f'Recovery: Failed to initialize microphone with sample rate {sample_rate}: {e}')
                    continue
            
            if not microphone_initialized:
                self.logger.error('Recovery: Failed to recover microphone with any sample rate')
                self.microphone = None
                return False
            
            return True
            
        except Exception as e:
            self.logger.error(f'Error during microphone recovery: {e}')
            self.microphone = None
            return False
