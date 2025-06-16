#!/usr/bin/env python3

"""
Audio Hardware Interface for Nevil-picar v2.0.

This module provides a thread-safe hardware interface for audio input/output
on the Nevil-picar v2.0 system, interfacing with the microphone and speaker hardware.
Based on the original Nevil v1.0 implementation.

FIXED VERSION: Uses OpenAI TTS as primary, robot_hat Music only for audio playback.
"""

import os
import threading
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
from io import BytesIO
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv()

# Try to import OpenAI for cloud-based Whisper API
try:
    from openai import OpenAI
    OPENAI_AVAILABLE = True
except ImportError:
    OPENAI_AVAILABLE = False

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

# Import required libraries, with fallback for when they're not available
try:
    import speech_recognition as sr
    import pyaudio
    import wave, sys
    
    # Try to import robot_hat Music and Pin for audio playback if available
    try:
        from robot_hat import Music, Pin
        ROBOT_HAT_AVAILABLE = True
    except ImportError as e:
        ROBOT_HAT_AVAILABLE = False
        # Import pyttsx3 as fallback if robot_hat is not available
        try:
            import pyttsx3
            PYTTSX3_AVAILABLE = True
        except ImportError:
            PYTTSX3_AVAILABLE = False
    
    # Try to import Whisper if available
    try:
        import whisper
        WHISPER_AVAILABLE = True
    except ImportError:
        WHISPER_AVAILABLE = False
        
    AUDIO_LIBS_AVAILABLE = True
except ImportError:
    WHISPER_AVAILABLE = False
    ROBOT_HAT_AVAILABLE = False
    PYTTSX3_AVAILABLE = False
    AUDIO_LIBS_AVAILABLE = False

def constrain(x, min_val, max_val):
    '''
    Constrains value to be within a range.
    '''
    return max(min_val, min(max_val, x))

class AudioHardwareInterface:
    """
    Audio hardware interface for Nevil-picar v2.0.
    
    This class provides a thread-safe interface to the microphone and speaker hardware,
    with proper mutex handling for real-time performance.
    Based on the original Nevil v1.0 implementation.
    
    FIXED: Uses OpenAI TTS as primary, robot_hat Music only for audio playback.
    """
    
    # Default configuration values - these will be overridden by environment variables if set
    DEFAULT_ENERGY_THRESHOLD = 300
    DEFAULT_PAUSE_THRESHOLD = 0.5
    DEFAULT_DYNAMIC_ENERGY = True
    DEFAULT_VOLUME_DB = 3
    DEFAULT_LANGUAGE = "en"
    DEFAULT_TTS_VOICE = "onyx"
    DEFAULT_SAMPLE_RATE = 16000
    DEFAULT_CHANNELS = 1
    DEFAULT_CHUNK_SIZE = 1024
    DEFAULT_FORMAT = None  # Will be set to pyaudio.paInt16 if available
    DEFAULT_WHISPER_MODEL = "small"  # Options: tiny, base, small, medium, large
    
    def __init__(self, node=None):
        """
        Initialize the audio hardware interface.
        
        Args:
            node: ROS2 node for logging (optional)
        """
        self.node = node
        self.logger = get_logger('audio_hardware_interface') if node is None else node.get_logger()
        
        # Initialize hardware mutex
        self.hardware_mutex = threading.Lock()
        self.speech_lock = threading.Lock()
        
        # Initialize state variables
        self.simulation_mode = False
        self.speech_loaded = False
        self.tts_file = None
        
        # Load configuration from environment variables
        self.language = get_env_var('SPEECH_RECOGNITION_LANGUAGE', self.DEFAULT_LANGUAGE)
        self.volume_db = float(get_env_var('SPEECH_SYNTHESIS_VOLUME', self.DEFAULT_VOLUME_DB))
        self.tts_voice = get_env_var('SPEECH_SYNTHESIS_VOICE', self.DEFAULT_TTS_VOICE)
        self.whisper_model = get_env_var('WHISPER_MODEL', self.DEFAULT_WHISPER_MODEL)
        
        # Load OpenAI API key for TTS and STT services
        self.openai_api_key = get_env_var('OPENAI_API_KEY', None)
        if self.openai_api_key:
            os.environ["OPENAI_API_KEY"] = self.openai_api_key
            self.logger.info('OpenAI API key loaded from environment for TTS and STT services')
        elif "OPENAI_API_KEY" in os.environ:
            self.openai_api_key = os.environ["OPENAI_API_KEY"]
            self.logger.info('Using OpenAI API key from system environment for TTS and STT services')
        else:
            self.logger.warn('No OpenAI API key found. OpenAI TTS and STT services will not be available.')
        
        # Check if audio libraries are available
        if not AUDIO_LIBS_AVAILABLE:
            self.logger.error('Audio libraries are not available')
            self.logger.warn('Running in simulation mode')
            self.recognizer = None
            self.microphone = None
            self.tts = None
            self.audio = None
            self.music_player = None
            self.speaker_pin = None
            self.simulation_mode = True
        else:
            # Initialize the audio hardware
            try:
                with self.hardware_mutex:
                    self.logger.info('Initializing audio hardware...')
                    
                    # Initialize speech recognition with environment variables
                    self.recognizer = sr.Recognizer()
                    self.recognizer.energy_threshold = int(get_env_var('SPEECH_RECOGNITION_ENERGY_THRESHOLD', self.DEFAULT_ENERGY_THRESHOLD))
                    self.recognizer.pause_threshold = float(get_env_var('SPEECH_RECOGNITION_PAUSE_THRESHOLD', self.DEFAULT_PAUSE_THRESHOLD))
                    self.recognizer.dynamic_energy_threshold = get_env_var('SPEECH_RECOGNITION_DYNAMIC_ENERGY', self.DEFAULT_DYNAMIC_ENERGY) in ['true', 'True', '1', 'yes', 'Yes']
                    
                    # Try to use the default microphone
                    try:
                        self.microphone = sr.Microphone()
                        self.logger.info('Microphone initialized successfully')
                    except Exception as e:
                        self.logger.error(f'Failed to initialize microphone: {e}')
                        self.microphone = None
                    
                    # Initialize TTS - FIXED: Use OpenAI TTS as primary, robot_hat Music only for playback
                    if OPENAI_AVAILABLE and self.openai_api_key:
                        # Primary TTS: OpenAI TTS
                        self.tts = "openai_tts"  # String marker for OpenAI TTS
                        self.logger.info('Using OpenAI TTS as primary text-to-speech engine')
                        
                        # Initialize robot_hat Music for audio playback if available
                        if ROBOT_HAT_AVAILABLE:
                            try:
                                self.logger.info('Initializing Robot HAT Music for audio playback...')
                                # Enable robot_hat speaker switch using Pin class
                                self.speaker_pin = Pin(20)
                                self.speaker_pin.on()  # Enable speaker
                                self.music_player = Music()
                                self.logger.info('Robot HAT Music initialized successfully for audio playback')
                            except Exception as e:
                                self.logger.error(f'Failed to initialize Robot HAT Music for playback: {e}')
                                self.music_player = None
                                self.speaker_pin = None
                        else:
                            self.music_player = None
                            self.speaker_pin = None
                            self.logger.info('Robot HAT not available, will use pygame for audio playback')
                    elif PYTTSX3_AVAILABLE:
                        try:
                            import pyttsx3
                            self.tts = pyttsx3.init()
                            self.tts.setProperty('rate', int(get_env_var('SPEECH_SYNTHESIS_RATE', 200)))
                            self.tts.setProperty('volume', float(get_env_var('SPEECH_SYNTHESIS_VOLUME', 1.0)))
                            self.logger.info('Pyttsx3 TTS initialized successfully')
                            self.music_player = None
                            self.speaker_pin = None
                        except Exception as e:
                            self.logger.error(f'Failed to initialize pyttsx3 TTS: {e}')
                            self.tts = None
                            self.music_player = None
                            self.speaker_pin = None
                    else:
                        self.logger.warn('No TTS engine available (neither OpenAI nor pyttsx3)')
                        self.tts = None
                        self.music_player = None
                        self.speaker_pin = None
                    
                    # Initialize PyAudio for more direct audio control
                    try:
                        self.audio = pyaudio.PyAudio()
                        self.logger.info('PyAudio initialized successfully')
                    except Exception as e:
                        self.logger.error(f'Failed to initialize PyAudio: {e}')
                        self.audio = None
                    
                    self.logger.info('Audio hardware initialization completed')
            except Exception as e:
                self.logger.error(f'Failed to initialize audio hardware: {e}')
                # Create mock audio interfaces for simulation if hardware initialization fails
                self.recognizer = None
                self.microphone = None
                self.tts = None
                self.audio = None
                self.music_player = None
                self.speaker_pin = None
                self.simulation_mode = True
                self.logger.warn('Running in simulation mode')
            else:
                # FIXED: Check if we have TTS (either OpenAI or pyttsx3)
                if self.recognizer and (self.microphone or self.audio) and self.tts:
                    self.simulation_mode = False
                    self.logger.info('Running in hardware mode')
                else:
                    self.simulation_mode = True
                    self.logger.info(f'mic:{self.microphone} rec: {self.recognizer}  tts: {self.tts}')
                    self.logger.warn('Some hardware components failed to initialize, running in partial simulation mode')
        
        # Configure default audio parameters
        self.sample_rate = self.DEFAULT_SAMPLE_RATE
        self.channels = self.DEFAULT_CHANNELS
        self.chunk_size = self.DEFAULT_CHUNK_SIZE
        self.format = pyaudio.paInt16 if AUDIO_LIBS_AVAILABLE else None

    def speak_text(self, text, voice=None, wait=True):
        """
        Speak text with proper mutex handling.
        
        Args:
            text: Text to speak
            voice: Voice to use (optional, defaults to self.tts_voice)
            wait: Whether to wait for speech to complete (default: True)
        """
        if self.simulation_mode:
            # In simulation mode, just log the text
            self.logger.debug(f'Simulation: Speaking text: {text}')
            return
        
        # In hardware mode, speak text with mutex protection
        try:
            with self.hardware_mutex:
                self.logger.debug(f'Speaking text: {text}')
                
                # Use voice parameter if provided, otherwise use default
                tts_voice = voice if voice else self.tts_voice
                
                # FIXED: Use OpenAI TTS as primary method
                if self.tts == "openai_tts" and OPENAI_AVAILABLE and self.openai_api_key:
                    try:
                        self.logger.debug(f'Using OpenAI TTS with voice: {tts_voice}')
                        
                        # Create OpenAI client
                        client = OpenAI(api_key=self.openai_api_key)
                        
                        # Create temp directory if it doesn't exist
                        import os
                        temp_dir = os.path.join(os.getcwd(), "tts_temp")
                        os.makedirs(temp_dir, exist_ok=True)
                        
                        # Create unique filename
                        import uuid
                        unique_id = str(uuid.uuid4())
                        output_file = os.path.join(temp_dir, f"tts_{unique_id}.mp3")
                        
                        # Generate speech using OpenAI TTS API with streaming response
                        with client.audio.speech.with_streaming_response.create(
                            model="tts-1",
                            voice="onyx",  # Hardcode to onyx as in v1.0
                            input=text,
                            response_format="mp3",
                            speed=0.9
                        ) as response:
                            response.stream_to_file(output_file)
                        
                        self.logger.debug(f'TTS audio saved to {output_file}')
                        
                        # Play the audio file using robot_hat Music if available
                        if self.music_player:
                            try:
                                self.logger.debug('Playing audio with Robot HAT Music')
                                self.music_player.music_play(output_file)
                                while self.music_player.pygame.mixer.music.get_busy():
                                    time.sleep(0.1)
                                self.music_player.music_stop()
                            except Exception as e:
                                self.logger.error(f'Failed to play with Robot HAT Music: {e}')
                                # Fallback to pygame
                                try:
                                    import pygame
                                    pygame.mixer.init()
                                    pygame.mixer.music.load(output_file)
                                    pygame.mixer.music.play()
                                    while pygame.mixer.music.get_busy():
                                        time.sleep(0.1)
                                    pygame.mixer.quit()
                                except ImportError:
                                    self.logger.error('Neither Robot HAT Music nor pygame available for mp3 playback')
                        else:
                            # Use pygame as fallback
                            try:
                                import pygame
                                pygame.mixer.init()
                                pygame.mixer.music.load(output_file)
                                pygame.mixer.music.play()
                                while pygame.mixer.music.get_busy():
                                    time.sleep(0.1)
                                pygame.mixer.quit()
                            except ImportError:
                                self.logger.error('Pygame not available for mp3 playback')
                        
                        # Clean up temporary file
                        try:
                            os.remove(output_file)
                        except Exception as e:
                            self.logger.error(f'Failed to remove temporary file: {e}')
                        
                        return  # Successfully used OpenAI TTS
                    except Exception as e:
                        self.logger.error(f'Failed to use OpenAI TTS: {e}')
                        self.logger.debug('Falling back to other TTS methods')
                        # Continue to fallback methods
                
                # Fallback to pyttsx3
                if hasattr(self.tts, 'say'):
                    if voice and hasattr(self.tts, 'getProperty'):
                        # Try to set the voice if specified
                        voices = self.tts.getProperty('voices')
                        for v in voices:
                            if voice in v.id:
                                self.tts.setProperty('voice', v.id)
                                break
                    
                    # Say the text
                    self.tts.say(text)
                    if wait:
                        self.tts.runAndWait()
                else:
                    self.logger.error('No working TTS engine available')
        except Exception as e:
            self.logger.error(f'Failed to speak text: {e}')

    def cleanup(self):
        """
        Clean up audio hardware resources.
        """
        self.logger.info('Cleaning up audio hardware resources')
        
        if self.simulation_mode:
            # Nothing to clean up in simulation mode
            return
        
        # In hardware mode, clean up the audio hardware with mutex protection
        try:
            with self.hardware_mutex:
                # Clean up robot_hat Music
                if self.music_player:
                    try:
                        self.music_player.music_stop()
                    except Exception as e:
                        self.logger.error(f'Failed to clean up Robot HAT Music: {e}')
                
                # Clean up pyttsx3 TTS
                if hasattr(self.tts, 'stop'):
                    try:
                        self.tts.stop()
                    except Exception as e:
                        self.logger.error(f'Failed to clean up pyttsx3 TTS: {e}')
                
                # Clean up PyAudio
                if self.audio:
                    try:
                        self.audio.terminate()
                    except Exception as e:
                        self.logger.error(f'Failed to clean up PyAudio: {e}')
        except Exception as e:
            self.logger.error(f'Failed to clean up audio hardware resources: {e}')

if __name__ == "__main__":
    print("Fixed Audio Hardware Interface - Uses OpenAI TTS as primary, robot_hat Music only for playback")