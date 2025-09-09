#!/usr/bin/env python3

"""
Audio for Nevil-picar v2.0.   ---  DEPRECATED 9.8.2025 - use nevil_audio_record.py and nevil_audio_play.py instead
------------------------------------------------------

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
    #import pyaudio
    import wave, sys
    
    # Try to import robot_hat Music and Pin for audio playback if available
    try:
        from robot_hat import Music, Pin
        ROBOT_HAT_AVAILABLE = True
    except ImportError as e:
        ROBOT_HAT_AVAILABLE = False
        # # Import pyttsx3 as fallback if robot_hat is not available
        # try:
        #     import pyttsx3
        #     PYTTSX3_AVAILABLE = True
        # except ImportError:
        #     PYTTSX3_AVAILABLE = False
    
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
    #PYTTSX3_AVAILABLE = False
    AUDIO_LIBS_AVAILABLE = False

def constrain(x, min_val, max_val):
    '''
    Constrains value to be within a range.
    '''
    return max(min_val, min(max_val, x))

class AudioHardwareInterface:
    """
    Audio for Nevil-picar v2.0.
    
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
        Initialize the Audio.
        
        Args:
            node: ROS2 node for logging (optional)
        """
        self.node = node
        self.logger = get_logger('audio_hardware_interface') if node is None else node.get_logger()
        
        # Add debug logging for initialization
        self.logger.warning('🔊 Audio: Initializing...')
        print("🔊 Audio: Initializing...")
        
        # Initialize hardware mutex
        self.hardware_mutex = threading.Lock()
        self.speech_lock = threading.Lock()
        
        # Initialize state variables
        self.simulation_mode = False
        self.speech_loaded = False
        self.tts_file = None
        self.audio = None

        # Configure default audio parameters
        self.sample_rate = self.DEFAULT_SAMPLE_RATE
        self.channels = self.DEFAULT_CHANNELS
        self.chunk_size = self.DEFAULT_CHUNK_SIZE
        self.format = None #pyaudio.paInt16 if AUDIO_LIBS_AVAILABLE else None
        
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
        self.logger.warning(f'🔊 Audio: AUDIO_LIBS_AVAILABLE = {AUDIO_LIBS_AVAILABLE}')
        print(f"🔊 Audio: AUDIO_LIBS_AVAILABLE = {AUDIO_LIBS_AVAILABLE}")
        
        if not AUDIO_LIBS_AVAILABLE:
            self.logger.error('🔊 Audio: Audio libraries are not available')
            print("🔊 Audio: Audio libraries are not available")
            self.logger.warn('🔊 Audio: Running in simulation mode')
            print("🔊 Audio: Running in simulation mode")
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
                    self.logger.warning('🔊 Audio: Initializing audio hardware...')
                    print("🔊 Audio: Initializing audio hardware...")
                    
                    # Initialize speech recognition with environment variables
                    self.recognizer = sr.Recognizer()
                    self.logger.warning("🔊 Audio: [Initializing Audio]sr.Recognizer()")
                    print("🔊 Audio: [Initializing Audio]sr.Recognizer()")

                    self.recognizer.energy_threshold = int(get_env_var('SPEECH_RECOGNITION_ENERGY_THRESHOLD', self.DEFAULT_ENERGY_THRESHOLD))
                    self.recognizer.pause_threshold = float(get_env_var('SPEECH_RECOGNITION_PAUSE_THRESHOLD', self.DEFAULT_PAUSE_THRESHOLD))
                    self.recognizer.dynamic_energy_threshold = get_env_var('SPEECH_RECOGNITION_DYNAMIC_ENERGY', self.DEFAULT_DYNAMIC_ENERGY) in ['true', 'True', '1', 'yes', 'Yes']
                    #self.energy_threshold = 300  # minimum audio energy to consider for recording
                    # self.dynamic_energy_threshold = True
                    # self.dynamic_energy_adjustment_damping = 0.15
                    # self.dynamic_energy_ratio = 1.5
                    # self.pause_threshold = 0.8  # seconds of non-speaking audio before a phrase is considered complete
                    # self.operation_timeout = None  # seconds after an internal operation (e.g., an API request) starts before it times out, or ``None`` for no timeout

                    # self.phrase_threshold = 0.3  # minimum seconds of speaking audio before we consider the speaking audio a phrase - values below this are ignored (for filtering out clicks and pops)
                    # self.non_speaking_duration = 0.5  # seconds of non-speaking audio to keep on both sides of the recording
                    self.logger.warning("🔊 Audio: [Initializing Audio]sr.Microphone")
                    print("🔊 Audio: [Initializing Audio]sr.Microphone")
                    # Try to use the default microphone with fallback sample rates
                    sample_rates_to_try = [44100, 48000, 16000, 22050, 8000]
                    microphone_initialized = False
                    
                    for sample_rate in sample_rates_to_try:
                        try:
                            self.logger.warning(f'🔊 Audio: Attempting to create sr.Microphone with sample rate {sample_rate}...')
                            print(f"🔊 Audio: Attempting to create sr.Microphone with sample rate {sample_rate}...")
                            self.microphone = sr.Microphone(device_index=1, #default device
                                  chunk_size=self.chunk_size, sample_rate=sample_rate)
                            self.sample_rate = sample_rate  # Update the sample rate to what worked
                            self.logger.warning(f'🔊 Audio: Microphone initialized successfully with sample rate {sample_rate}')
                            print(f"🔊 Audio: Microphone initialized successfully with sample rate {sample_rate}")
                            self.logger.warning(f'🔊 Audio: Microphone object: {self.microphone}')
                            print(f"🔊 Audio: Microphone object: {self.microphone}")
                            self.logger.warning(f'🔊 Audio: Microphone type: {type(self.microphone)}')
                            print(f"🔊 Audio: Microphone type: {type(self.microphone)}")
                            microphone_initialized = True
                            break
                        except Exception as e:
                            self.logger.warning(f'🔊 Audio: Failed to initialize microphone with sample rate {sample_rate}: {e}')
                            print(f"🔊 Audio: Failed to initialize microphone with sample rate {sample_rate}: {e}")
                            continue
                    
                    if not microphone_initialized:
                        self.logger.error('🔊 Audio: Failed to initialize microphone with any sample rate')
                        print("🔊 Audio: Failed to initialize microphone with any sample rate")
                        self.logger.error('🔊 Audio: Setting microphone to None due to microphone initialization failure')
                        print("🔊 Audio: Setting microphone to None due to microphone initialization failure")
                        self.microphone = None
                    
                    # Initialize TTS - FIXED: Use OpenAI TTS as primary, robot_hat Music only for playback
                    self.logger.info(f'OPENAI_AVAILABLE: {OPENAI_AVAILABLE} self.openai_api_key: {self.openai_api_key[3:][:3]}')
                    if OPENAI_AVAILABLE and self.openai_api_key:
                        # Primary TTS: OpenAI TTS
                        self.tts = "openai_tts"  # String marker for OpenAI TTS
                        self.logger.info('Using OpenAI TTS as primary text-to-speech engine')
                        
                        # Initialize robot_hat Music for audio playback if available
                        if ROBOT_HAT_AVAILABLE:
                            try:
                                self.logger.info('Initializing Robot HAT Music for audio playback...')
                                # Enable robot_hat speaker switch using Pin class
                                try:
                                    os.popen("pinctrl set 20 op dh")  # enable robot_hat speaker switch
                                    self.logger.info(f'os.popen: speaker switch enabled')
                                except Exception as pin_error:
                                    self.logger.warn(f'Failed to enable speaker switch: {pin_error}')
                                
                                # self.speaker_pin = Pin(20)
                                # self.speaker_pin.on()  # Enable speaker
                                self.music_player = Music()#device_name="hw:3,0")
                                self.logger.info('Robot HAT Music initialized successfully for audio playback')
                            except Exception as e:
                                self.logger.error(f'Failed to initialize Robot HAT Music for playback: {e}')
                                self.music_player = None
                                self.speaker_pin = None
                        else:
                            self.music_player = None
                            self.speaker_pin = None
                            self.logger.info('Robot HAT not available, will use pygame for audio playback')
                    # elif PYTTSX3_AVAILABLE:
                    #     try:
                    #         import pyttsx3
                    #         self.tts = pyttsx3.init()
                    #         self.tts.setProperty('rate', int(get_env_var('SPEECH_SYNTHESIS_RATE', 200)))
                    #         self.tts.setProperty('volume', float(get_env_var('SPEECH_SYNTHESIS_VOLUME', 1.0)))
                    #         self.logger.info('Pyttsx3 TTS initialized successfully')
                    #         self.music_player = None
                    #         self.speaker_pin = None
                    #     except Exception as e:
                    #         self.logger.error(f'Failed to initialize pyttsx3 TTS: {e}')
                    #         self.tts = None
                    #         self.music_player = None
                    #         self.speaker_pin = None
                    else:
                        self.logger.warn('No TTS engine available (neither OpenAI nor pyttsx3)')
                        self.tts = None
                        self.music_player = None
                        self.speaker_pin = None
                    
                    # # Initialize PyAudio for more direct audio control
                    # try:
                    #     self.audio = pyaudio.PyAudio()
                    #     self.logger.info('PyAudio initialized successfully')
                    # except Exception as e:
                    #     self.logger.error(f'Failed to initialize PyAudio: {e}')
                    #     self.audio = None
                    
                    self.logger.info('Audio hardware initialization completed')
            except Exception as e:
                self.logger.error(f'Failed to initialize audio hardware: {e}')
                self.logger.error(f'Exception type: {type(e).__name__}')
                self.logger.error(f'Exception details: {e}')
                # Create mock audio interfaces for simulation if hardware initialization fails
                self.recognizer = None
                # Don't set microphone to None if it was successfully initialized
                # self.microphone = None  # Only set to None if microphone init itself fails
                self.tts = None
                self.audio = None
                self.music_player = None
                self.speaker_pin = None
                self.simulation_mode = True
                self.logger.warn('Running in simulation mode')
            # else:
            #     # FIXED: Check if we have TTS (either OpenAI or pyttsx3)
            #     if self.recognizer and self.microphone and self.tts:
            #         self.simulation_mode = False
            #         self.logger.info('Running in hardware mode')
            #     else:
            #         self.simulation_mode = True
            #         self.logger.info(f'mic:{self.microphone} rec: {self.recognizer}  tts: {self.tts}')
            #         self.logger.warn('Some hardware components failed to initialize, running in partial simulation mode')

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
                self.logger.warning(f'🔊 AUDIO HARDWARE: Speaking text: {text}')
                print(f"🔊 AUDIO HARDWARE: Speaking text: {text}")
                
                # Use voice parameter if provided, otherwise use default
                tts_voice = voice if voice else self.tts_voice
                
                # FIXED: Use OpenAI TTS as primary method
                if self.tts == "openai_tts" and OPENAI_AVAILABLE and self.openai_api_key:
                    try:
                        self.logger.debug(f'Using OpenAI TTS with voice: {tts_voice}')
                        
                        # Create OpenAI client with timeout
                        client = OpenAI(api_key=self.openai_api_key, timeout=30.0)
                        
                        # Create temp directory if it doesn't exist
                        import os
                        temp_dir = os.path.join(os.getcwd(), "tts_temp")
                        os.makedirs(temp_dir, exist_ok=True)
                        
                        # Clean up old temp files (older than 5 minutes)
                        import glob
                        import time as time_module
                        for old_file in glob.glob(os.path.join(temp_dir, "tts_*.mp3")):
                            try:
                                if time_module.time() - os.path.getmtime(old_file) > 300:  # 5 minutes
                                    os.remove(old_file)
                            except:
                                pass
                        
                        # Create unique filename
                        import uuid
                        unique_id = str(uuid.uuid4())
                        output_file = os.path.join(temp_dir, f"tts_{unique_id}.mp3")
                        
                        # Generate speech using OpenAI TTS API with streaming response and error handling
                        try:
                            with client.audio.speech.with_streaming_response.create(
                                model="tts-1",
                                voice="onyx",  # Hardcode to onyx as in v1.0
                                input=text[:4000],  # Limit text length to avoid API errors
                                response_format="mp3",
                                speed=0.9
                            ) as response:
                                response.stream_to_file(output_file)
                        except Exception as api_error:
                            self.logger.error(f'OpenAI TTS API error: {api_error}')
                            # Clean up partial file
                            if os.path.exists(output_file):
                                try:
                                    os.remove(output_file)
                                except:
                                    pass
                            raise api_error
                        
                        self.logger.debug(f'TTS audio saved to {output_file}')
                        
                        # Use non-realtime aplay command to prevent ALSA underruns
                        try:
                            self.logger.warning('🔊 AUDIO PLAYBACK: Starting aplay (non-realtime)')
                            print("🔊 AUDIO PLAYBACK: Starting mpg123 (non-realtime)")
                            
                            # Use mpg123 for MP3 playback - most stable approach
                            import subprocess
                            
                            # Create environment with our ALSA config
                            play_env = os.environ.copy()
                            if os.path.exists('/tmp/nevil_asoundrc'):
                                play_env['ALSA_CONFIG_FILE'] = '/tmp/nevil_asoundrc'
                            
                            cmd = ['mpg123', '-q', output_file]  # -q for quiet mode
                            
                            self.logger.warning(f'🔊 AUDIO PLAYBACK: Executing command: {" ".join(cmd)}')
                            print(f"🔊 AUDIO PLAYBACK: Executing command: {' '.join(cmd)}")
                            
                            # Play and wait for completion
                            result = subprocess.run(cmd, env=play_env, capture_output=True, text=True, timeout=30)
                            
                            if result.returncode == 0:
                                self.logger.warning('🔊 AUDIO PLAYBACK: mpg123 completed successfully')
                                print("🔊 AUDIO PLAYBACK: mpg123 completed successfully")
                            else:
                                self.logger.error(f'🔊 AUDIO PLAYBACK: mpg123 failed: {result.stderr}')
                                print(f"🔊 AUDIO PLAYBACK: mpg123 failed: {result.stderr}")
                                # Fallback to robot_hat Music if mpg123 fails
                                raise Exception(f"mpg123 failed: {result.stderr}")
                            
                        except Exception as mpg123_error:
                            self.logger.warning(f'🔊 AUDIO PLAYBACK: mpg123 failed, falling back to robot_hat Music: {mpg123_error}')
                            print(f"🔊 AUDIO PLAYBACK: mpg123 failed, falling back to robot_hat Music: {mpg123_error}")
                            
                            # Fallback to robot_hat Music if mpg123 fails
                            if self.music_player:
                                try:
                                    self.logger.warning(f'🔊 AUDIO PLAYBACK: Fallback - Playing with Robot HAT Music: {output_file}')
                                    print(f"🔊 AUDIO PLAYBACK: Fallback - Playing with Robot HAT Music: {output_file}")
                                    
                                    #VOLUME MAX
                                    self.music_player.pygame.mixer.music.set_volume(1.0)

                                    # Add silent audio warm-up to prevent static
                                    self.logger.info('Playing silent audio warm-up to prevent static')
                                    try:
                                        # Generate 3.0 seconds of silence at 44.1kHz for better warm-up
                                        import numpy as np
                                        silence_duration = 3.0  # Increased from 0.1 to 3.0 seconds
                                        sample_rate = 44100
                                        silence_samples = int(silence_duration * sample_rate)
                                        silence_data = np.zeros(silence_samples, dtype=np.int16)
                                        
                                        # Save silence to temporary file
                                        import tempfile
                                        with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as silence_file:
                                            import wave
                                            with wave.open(silence_file.name, 'w') as wav_file:
                                                wav_file.setnchannels(1)  # Mono
                                                wav_file.setsampwidth(2)  # 16-bit
                                                wav_file.setframerate(sample_rate)
                                                wav_file.writeframes(silence_data.tobytes())
                                        
                                        # Play silence multiple times for better initialization
                                        for i in range(2):  # Play twice
                                            self.logger.info(f'Playing silence warm-up {i+1}/2')
                                            self.music_player.music_play(silence_file.name)
                                            time.sleep(silence_duration + 0.5)  # Wait for completion + buffer
                                            self.music_player.music_stop()
                                            time.sleep(0.3)  # Brief pause between plays
                                        
                                        # Clean up silence file
                                        os.unlink(silence_file.name)
                                        self.logger.info('Silent audio warm-up completed')
                                    except Exception as silence_error:
                                        self.logger.warning(f'Silent audio warm-up failed: {silence_error}')

                                    self.music_player.music_play(output_file)
                                    self.logger.warning('🔊 AUDIO PLAYBACK: Robot HAT Music started playing')
                                    print("🔊 AUDIO PLAYBACK: Robot HAT Music started playing")
                                    
                                    # Wait for playback with timeout to prevent hanging
                                    max_wait_time = 30  # Maximum 30 seconds
                                    wait_start = time.time()
                                    while self.music_player.pygame.mixer.music.get_busy():
                                        if time.time() - wait_start > max_wait_time:
                                            self.logger.warning('🔊 AUDIO PLAYBACK: Timeout, stopping')
                                            print("🔊 AUDIO PLAYBACK: Timeout, stopping")
                                            break
                                        time.sleep(0.1)
                                    
                                    self.music_player.music_stop()
                                    self.logger.warning('🔊 AUDIO PLAYBACK: Robot HAT Music completed')
                                    print("🔊 AUDIO PLAYBACK: Robot HAT Music completed")
                                except Exception as e:
                                    self.logger.error(f'Failed to play with Robot HAT Music: {e}')
                                    # Fallback to pygame
                                    try:
                                        import pygame
                                        import os
                                        # # Configure pygame to use HifiBerry DAC (card 3) with proper ALSA config
                                        # os.environ['SDL_AUDIODRIVER'] = 'alsa'
                                        # os.environ['ALSA_CARD'] = '3'
                                        # os.environ['ALSA_DEVICE'] = '0'
                                        # Use our custom ALSA config if available
                                        # if os.path.exists('/tmp/nevil_asoundrc'):
                                        #     os.environ['ALSA_CONFIG_FILE'] = '/tmp/nevil_asoundrc'
                                        pygame.mixer.quit()  # Clean up any existing mixer
                                        pygame.mixer.init(frequency=44100, size=-16, channels=2, buffer=4096)
                                        pygame.mixer.music.load(output_file)
                                        pygame.mixer.music.set_volume(1.0)
                                        pygame.mixer.music.play()
                                        
                                        # Wait with timeout
                                        max_wait_time = 30
                                        wait_start = time.time()
                                        while pygame.mixer.music.get_busy():
                                            if time.time() - wait_start > max_wait_time:
                                                self.logger.warning('Pygame playback timeout, stopping')
                                                pygame.mixer.music.stop()
                                                break
                                            time.sleep(0.1)
                                        
                                        pygame.mixer.quit()
                                        self.logger.debug('Pygame fallback playback completed')
                                    except Exception as pygame_error:
                                        self.logger.error(f'Pygame fallback also failed: {pygame_error}')
                        
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
                    self.logger.error('Microphone is None - cannot listen for speech')
                    self.logger.error('Microphone object failed during initialization or became corrupted')
                    return None
                
                # Additional safety check - ensure microphone is not None
                if self.microphone is None:
                    self.logger.error('Microphone is None after mutex check - cannot listen for speech')
                    self.logger.error('Microphone object became None during operation - possible corruption')
                    return None
                
                try:
                    with self.microphone as source:
                        self.logger.debug('Listening for speech...')
                        
                        # Set energy threshold based on environment
                        if self.recognizer.energy_threshold < self.DEFAULT_ENERGY_THRESHOLD:
                            self.recognizer.energy_threshold = self.DEFAULT_ENERGY_THRESHOLD
                        
                        if adjust_for_ambient_noise:
                            try:
                                self.logger.debug('Adjusting for ambient noise...')
                                self.recognizer.adjust_for_ambient_noise(source, duration=0.5)
                                self.logger.debug('adjust_for_ambient_noise: done')
                            except Exception as e:
                                self.logger.warning(f'PyAudio read failed during ambient noise adjustment: {e}')
                                # PyAudio failure - mark microphone as corrupted and return None
                                self.logger.error('Microphone corrupted by PyAudio failure - disabling speech recognition')
                                self.microphone = None
                                return None
                        
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
            self.logger.error(f'Failed to listen for speech: {e}')
            return None

    def whisper_API_stt(self, audio, language=None):
        """
        Speech to Text using OpenAI's Whisper API.
        
        Args:
            audio: Audio data to transcribe
            language: Language code (defaults to self.language if None)
            
        Returns:
            Transcribed text or False if failed
        """
        if self.simulation_mode or audio is None:
            # In simulation mode, return mock text
            self.logger.debug('Simulation: STT with OpenAI Whisper API')
            return "This is simulated speech to text"
        
        # Use default language if not specified
        if language is None:
            language = self.language
        
        # Check if OpenAI API is available
        if not OPENAI_AVAILABLE or not self.openai_api_key:
            self.logger.error('OpenAI API not available or API key not set')
            return False
        
        try:
            # Create OpenAI client
            client = OpenAI(api_key=self.openai_api_key)
            
            # Prepare audio data for OpenAI API
            wav_data = BytesIO(audio.get_wav_data())
            wav_data.name = "speech.wav"
            
            # Call OpenAI Whisper API
            transcript = client.audio.transcriptions.create(
                model="whisper-1",
                file=wav_data,
                language=language if len(language) == 2 else None,
                prompt="this is the conversation between me and a robot"
            )
            
            text = transcript.text.strip()
            self.logger.debug(f'OpenAI Whisper API transcribed: {text}')
            return text
            
        except Exception as e:
            self.logger.error(f'STT error with OpenAI Whisper API: {e}')
            return False

    def recognize_speech(self, audio, language=None, use_online=True, api='auto'):
        """
        Recognize speech with proper mutex handling.
        
        Args:
            audio: Audio data to recognize
            language: Language code (defaults to self.language if None)
            use_online: Whether to use online recognition
            api: API to use for online recognition ('openai', 'whisper-local', 'auto', etc.)
        
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
            
        # Convert language code format if needed
        lang_code = language
        if len(language) == 2:  # Convert 'en' to 'en-US' format for some APIs
            if language == 'en':
                lang_code = 'en-US'
            elif language == 'fr':
                lang_code = 'fr-FR'
            elif language == 'es':
                lang_code = 'es-ES'
            elif language == 'de':
                lang_code = 'de-DE'
            elif language == 'it':
                lang_code = 'it-IT'
            elif language == 'ja':
                lang_code = 'ja-JP'
            elif language == 'ko':
                lang_code = 'ko-KR'
            elif language == 'zh':
                lang_code = 'zh-CN'
        
        # In hardware mode, recognize speech with mutex protection
        try:
            with self.hardware_mutex:
                # Try OpenAI Whisper API first for 'auto' or 'openai' api options
                if (api == 'auto' or api == 'openai') and OPENAI_AVAILABLE and self.openai_api_key and use_online:
                    self.logger.debug('Using OpenAI Whisper API as primary STT method')
                    result = self.whisper_API_stt(audio, language)
                    if result:
                        return result
                    else:
                        self.logger.debug('OpenAI Whisper API failed, falling back to other methods')
                        # If api was specifically 'openai', don't continue to fallbacks
                        if api == 'openai':
                            return ""
                
                # If all methods fail, return empty string
                self.logger.error('All speech recognition methods failed')
                return ""
        except sr.UnknownValueError:
            self.logger.debug('Speech not recognized')
            return ""
        except sr.RequestError as e:
            self.logger.error(f'Error with speech recognition service: {e}')
            if use_online and api != 'sphinx':
                self.logger.debug('Falling back to offline recognition')
                try:
                    return self.recognize_speech(audio, language, False, 'sphinx')
                except:
                    return ""
            return ""
        except Exception as e:
            self.logger.error(f'Failed to recognize speech: {e}')
            return ""

    def set_speech_recognition_parameters(self, energy_threshold=None, pause_threshold=None, dynamic_energy=None,
                                        dynamic_energy_adjustment_damping=None, dynamic_energy_ratio=None,
                                        operation_timeout=None, phrase_threshold=None, non_speaking_duration=None):
        """
        Set speech recognition parameters with proper mutex handling.
        
        Args:
            energy_threshold: Energy level threshold for considering an audio frame as speech (50-4000 range)
            pause_threshold: Seconds of non-speaking audio before a phrase is considered complete
            dynamic_energy: Whether to dynamically adjust the energy threshold based on ambient noise
            dynamic_energy_adjustment_damping: Controls adaptation rate of energy threshold (0-1 range)
            dynamic_energy_ratio: How much louder speech must be vs ambient noise
            operation_timeout: Maximum seconds to wait for speech input before timeout
            phrase_threshold: Minimum seconds of speaking audio before considering it a phrase
            non_speaking_duration: Seconds of non-speaking audio to keep on both sides of recording
        """
        if self.simulation_mode or not self.recognizer:
            # In simulation mode, just log the parameters
            self.logger.debug(f'Simulation: Setting speech recognition parameters')
            return
        
        # In hardware mode, set the parameters with mutex protection
        try:
            with self.hardware_mutex:
                if energy_threshold is not None:
                    self.recognizer.energy_threshold = energy_threshold
                    self.logger.debug(f'Set energy threshold to {energy_threshold}')
                
                if pause_threshold is not None:
                    self.recognizer.pause_threshold = pause_threshold
                    self.logger.debug(f'Set pause threshold to {pause_threshold}')
                
                if dynamic_energy is not None:
                    self.recognizer.dynamic_energy_threshold = dynamic_energy
                    self.logger.debug(f'Set dynamic energy to {dynamic_energy}')
                
                # Apply v1.0 advanced parameters if the recognizer supports them
                if dynamic_energy_adjustment_damping is not None and hasattr(self.recognizer, 'dynamic_energy_adjustment_damping'):
                    self.recognizer.dynamic_energy_adjustment_damping = dynamic_energy_adjustment_damping
                    self.logger.debug(f'Set dynamic energy adjustment damping to {dynamic_energy_adjustment_damping}')
                
                if dynamic_energy_ratio is not None and hasattr(self.recognizer, 'dynamic_energy_ratio'):
                    self.recognizer.dynamic_energy_ratio = dynamic_energy_ratio
                    self.logger.debug(f'Set dynamic energy ratio to {dynamic_energy_ratio}')
                
                if operation_timeout is not None and hasattr(self.recognizer, 'operation_timeout'):
                    self.recognizer.operation_timeout = operation_timeout
                    self.logger.debug(f'Set operation timeout to {operation_timeout}')
                
                if phrase_threshold is not None and hasattr(self.recognizer, 'phrase_threshold'):
                    self.recognizer.phrase_threshold = phrase_threshold
                    self.logger.debug(f'Set phrase threshold to {phrase_threshold}')
                
                if non_speaking_duration is not None and hasattr(self.recognizer, 'non_speaking_duration'):
                    self.recognizer.non_speaking_duration = non_speaking_duration
                    self.logger.debug(f'Set non-speaking duration to {non_speaking_duration}')
                
                self.logger.info('Speech recognition parameters updated with v1.0 configuration')
        except Exception as e:
            self.logger.error(f'Failed to set speech recognition parameters: {e}')

    def set_speaker_voice(self, voice_id):
        """
        Set the speaker voice for TTS.
        
        Args:
            voice_id: Voice ID to use for TTS
        """
        if voice_id:
            self.tts_voice = voice_id
            self.logger.debug(f'Set TTS voice to: {voice_id}')

    def set_speech_rate(self, rate):
        """
        Set the speech rate for TTS.
        
        Args:
            rate: Speech rate in words per minute
        """
        if self.tts and hasattr(self.tts, 'setProperty'):
            try:
                self.tts.setProperty('rate', rate)
                self.logger.debug(f'Set speech rate to: {rate}')
            except Exception as e:
                self.logger.error(f'Failed to set speech rate: {e}')

    def set_speech_volume(self, volume):
        """
        Set the speech volume for TTS.
        
        Args:
            volume: Volume level (0.0 to 1.0)
        """
        self.volume_db = volume
        if self.tts and hasattr(self.tts, 'setProperty'):
            try:
                self.tts.setProperty('volume', volume)
                self.logger.debug(f'Set speech volume to: {volume}')
            except Exception as e:
                self.logger.error(f'Failed to set speech volume: {e}')

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
                
                # Clean up PyAudio
                if self.audio:
                    try:
                        self.audio.terminate()
                        self.logger.info('PyAudio terminated')
                    except Exception as e:
                        self.logger.error(f'Failed to clean up PyAudio: {e}')
        except Exception as e:
            self.logger.error(f'Failed to clean up audio hardware resources: {e}')

if __name__ == "__main__":
    print("Fixed Audio - Uses OpenAI TTS as primary, robot_hat Music only for playback")
    print("Fixed Audio - Uses OpenAI TTS as primary, robot_hat Music only for playback")