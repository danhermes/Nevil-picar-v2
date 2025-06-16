#!/usr/bin/env python3

"""
Audio Hardware Interface for Nevil-picar v2.0.

This module provides a thread-safe hardware interface for audio input/output
on the Nevil-picar v2.0 system, interfacing with the microphone and speaker hardware.
Based on the original Nevil v1.0 implementation.
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
                    
                    # Initialize TTS
                    if ROBOT_HAT_AVAILABLE:
                        try:
                            self.logger.info('Attempting to initialize Robot HAT Music...')
                            
                            # Use a timeout to prevent hanging
                            import signal
                            
                            def timeout_handler(signum, frame):
                                raise TimeoutError("Robot HAT initialization timed out")
                            
                            # Set a 5-second timeout for robot_hat initialization
                            signal.signal(signal.SIGALRM, timeout_handler)
                            signal.alarm(5)
                            
                            try:
                                # Enable robot_hat speaker switch using Pin class
                                self.logger.debug('Creating Pin(20) for speaker control...')
                                speaker_pin = Pin(20)
                                self.logger.debug('Pin(20) created successfully')
                                
                                self.logger.debug('Enabling speaker pin...')
                                speaker_pin.on()  # Enable speaker
                                self.logger.debug('Speaker pin enabled successfully')
                                
                                self.logger.debug('Creating Music() object...')
                                self.tts = Music()
                                self.logger.info('Robot HAT Music initialized successfully for audio playback')
                                self.logger.info(f'TTS object type: {type(self.tts)}')
                                
                                # Cancel the alarm
                                signal.alarm(0)
                                
                            except TimeoutError:
                                self.logger.error('Robot HAT initialization timed out - hardware may not be available')
                                self.tts = None
                                signal.alarm(0)
                                
                        except Exception as e:
                            self.logger.error(f'Failed to initialize Robot HAT Music: {e}')
                            import traceback
                            self.logger.error(f'Traceback: {traceback.format_exc()}')
                            self.tts = None
                            # Make sure to cancel any pending alarm
                            try:
                                signal.alarm(0)
                            except:
                                pass
                    elif PYTTSX3_AVAILABLE:
                        try:
                            import pyttsx3
                            self.tts = pyttsx3.init()
                            self.tts.setProperty('rate', int(get_env_var('SPEECH_SYNTHESIS_RATE', 200)))
                            self.tts.setProperty('volume', float(get_env_var('SPEECH_SYNTHESIS_VOLUME', 1.0)))
                            self.logger.info('Pyttsx3 TTS initialized successfully')
                        except Exception as e:
                            self.logger.error(f'Failed to initialize pyttsx3 TTS: {e}')
                            self.tts = None
                    else:
                        self.logger.warn('No TTS engine available (neither robot_hat nor pyttsx3)')
                        self.tts = None
                    
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
                self.simulation_mode = True
                self.logger.warn('Running in simulation mode')
            else:
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
    
    def get_microphone_list(self):
        """
        Get a list of available microphones with proper mutex handling.
        
        Returns:
            List of available microphone devices
        """
        if self.simulation_mode:
            # In simulation mode, return a mock list
            return ["Simulated Microphone"]
        
        # In hardware mode, get the microphone list with mutex protection
        try:
            with self.hardware_mutex:
                mic_list = sr.Microphone.list_microphone_names()
                self.logger.debug(f'Available microphones: {mic_list}')
                return mic_list
        except Exception as e:
            self.logger.error(f'Failed to get microphone list: {e}')
            return []
    
    def get_speaker_list(self):
        """
        Get a list of available speakers with proper mutex handling.
        
        Returns:
            List of available speaker devices
        """
        if self.simulation_mode:
            # In simulation mode, return a mock list
            return ["Simulated Speaker"]
        
        # In hardware mode, get the speaker list with mutex protection
        try:
            with self.hardware_mutex:
                voices = self.tts_engine.getProperty('voices')
                voice_list = [voice.name for voice in voices]
                self.logger.debug(f'Available voices: {voice_list}')
                return voice_list
        except Exception as e:
            self.logger.error(f'Failed to get speaker list: {e}')
            return []
    
    def set_microphone_device(self, device_index):
        """
        Set the microphone device with proper mutex handling.
        
        Args:
            device_index: Index of the microphone device to use
        """
        if self.simulation_mode:
            # In simulation mode, just store the value
            self.logger.debug(f'Simulation: Setting microphone device to {device_index}')
            return
        
        # In hardware mode, set the microphone device with mutex protection
        try:
            with self.hardware_mutex:
                self.microphone = sr.Microphone(device_index=device_index)
                self.logger.info(f'Set microphone device to index {device_index}')
        except Exception as e:
            self.logger.error(f'Failed to set microphone device: {e}')
    
    def set_speaker_voice(self, voice_id):
        """
        Set the speaker voice with proper mutex handling.
        
        Args:
            voice_id: ID of the voice to use
        """
        if self.simulation_mode:
            # In simulation mode, just store the value
            self.logger.debug(f'Simulation: Setting speaker voice to {voice_id}')
            return
        
        # In hardware mode, set the speaker voice with mutex protection
        try:
            with self.hardware_mutex:
                voices = self.tts_engine.getProperty('voices')
                for voice in voices:
                    if voice_id in voice.id:
                        self.tts_engine.setProperty('voice', voice.id)
                        self.logger.info(f'Set speaker voice to {voice_id}')
                        break
        except Exception as e:
            self.logger.error(f'Failed to set speaker voice: {e}')
    
    def set_speech_rate(self, rate):
        """
        Set the speech rate with proper mutex handling.
        
        Args:
            rate: Speech rate (words per minute)
        """
        if self.simulation_mode:
            # In simulation mode, just store the value
            self.logger.debug(f'Simulation: Setting speech rate to {rate}')
            return
        
        # In hardware mode, set the speech rate with mutex protection
        try:
            with self.hardware_mutex:
                self.tts_engine.setProperty('rate', rate)
                self.logger.debug(f'Set speech rate to {rate}')
        except Exception as e:
            self.logger.error(f'Failed to set speech rate: {e}')
    
    def set_speech_volume(self, volume):
        """
        Set the speech volume with proper mutex handling.
        
        Args:
            volume: Volume level (0.0 to 1.0)
        """
        # Constrain volume to valid range
        volume = max(0.0, min(1.0, volume))
        
        if self.simulation_mode:
            # In simulation mode, just store the value
            self.logger.debug(f'Simulation: Setting speech volume to {volume}')
            return
        
        # In hardware mode, set the speech volume with mutex protection
        try:
            with self.hardware_mutex:
                self.tts_engine.setProperty('volume', volume)
                self.logger.debug(f'Set speech volume to {volume}')
        except Exception as e:
            self.logger.error(f'Failed to set speech volume: {e}')
    
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
                with self.microphone as source:
                    self.logger.debug('Listening for speech...')
                    if adjust_for_ambient_noise:
                        self.recognizer.adjust_for_ambient_noise(source, duration=0.5)
                    
                    # Set energy threshold based on environment
                    if self.recognizer.energy_threshold < self.DEFAULT_ENERGY_THRESHOLD:
                        self.recognizer.energy_threshold = self.DEFAULT_ENERGY_THRESHOLD
                    
                    audio = self.recognizer.listen(source, timeout=timeout, phrase_time_limit=phrase_time_limit)
                    self.logger.debug('Speech captured')
                    return audio
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
                
                # # Determine which API to use for fallback
                # if api == 'auto':
                #     # Auto-select the best available API for fallback
                #     if WHISPER_AVAILABLE:
                #         api = 'whisper-local'
                #     elif use_online:
                #         api = 'google'
                #     else:
                #         api = 'sphinx'
                
                # # Use the selected API for fallback
                # if api == 'whisper-local' and WHISPER_AVAILABLE:
                #     self.logger.debug('Using local Whisper for speech recognition')
                    
                #     # Save audio to a temporary file for Whisper
                #     import tempfile
                #     with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as temp_audio:
                #         temp_filename = temp_audio.name
                        
                #     # Write audio data to the temporary file
                #     with open(temp_filename, "wb") as f:
                #         f.write(audio.get_wav_data())
                    
                #     try:
                #         # Load Whisper model from environment variable
                #         model = whisper.load_model(self.whisper_model)
                        
                #         # Transcribe the audio
                #         result = model.transcribe(temp_filename, language=language if len(language) == 2 else None)
                #         text = result["text"].strip()
                        
                #         # Clean up the temporary file
                #         os.remove(temp_filename)
                #         return text
                #     except Exception as e:
                #         self.logger.error(f'Local Whisper transcription error: {e}')
                #         # Clean up the temporary file
                #         if os.path.exists(temp_filename):
                #             os.remove(temp_filename)
                #         # Continue to fallback methods
                
                # if api == 'google' and use_online:
                #     self.logger.debug('Using Google Speech Recognition')
                #     try:
                #         text = self.recognizer.recognize_google(audio, language=lang_code)
                #         return text
                #     except Exception as e:
                #         self.logger.error(f'Google Speech Recognition error: {e}')
                #         # Continue to fallback methods
                
                # if api == 'sphinx' or not use_online:
                #     self.logger.debug('Using Sphinx (offline) Speech Recognition')
                #     try:
                #         # Note: Sphinx might need specific language models installed
                #         text = self.recognizer.recognize_sphinx(audio)
                #         return text
                #     except Exception as e:
                #         self.logger.error(f'Sphinx Speech Recognition error: {e}')
                #         # Continue to fallback methods
                
                # # Final fallback - try any available method
                # self.logger.debug('Using fallback Speech Recognition methods')
                
                # # Try Google if online is allowed
                # if use_online:
                #     try:
                #         text = self.recognizer.recognize_google(audio, language=lang_code)
                #         self.logger.debug(f'Fallback Google recognized: {text}')
                #         return text
                #     except Exception:
                #         pass
                
                # # Try Sphinx as last resort
                # try:
                #     text = self.recognizer.recognize_sphinx(audio)
                #     self.logger.debug(f'Fallback Sphinx recognized: {text}')
                #     return text
                # except Exception:
                #     pass
                
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
                
                # Try OpenAI TTS (Whisper/Onyx) first if available
                if OPENAI_AVAILABLE and self.openai_api_key:
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
                        # This is the key to avoiding hanging - use streaming response
                        with client.audio.speech.with_streaming_response.create(
                            model="tts-1",
                            voice="onyx",  # Hardcode to onyx as in v1.0
                            input=text,
                            response_format="mp3",
                            speed=0.9
                        ) as response:
                            response.stream_to_file(output_file)
                        
                        self.logger.debug(f'TTS audio saved to {output_file}')
                        
                        # Play the audio file using pygame if available
                        try:
                            import pygame
                            pygame.mixer.init()
                            pygame.mixer.music.load(output_file)
                            pygame.mixer.music.play()
                            while pygame.mixer.music.get_busy():
                                time.sleep(0.1)
                            pygame.mixer.quit()
                        except ImportError:
                            self.logger.debug('Pygame not available, trying to play with Music class')
                            try:
                                from robot_hat import Music
                                music = Music()
                                music.music_play(output_file)
                                while music.pygame.mixer.music.get_busy():
                                    time.sleep(0.1)
                                music.music_stop()
                            except ImportError:
                                self.logger.error('Neither pygame nor robot_hat Music available for mp3 playback')
                                # Continue to fallback methods
                        except Exception as e:
                            self.logger.error(f'Failed to play mp3: {e}')
                            # Continue to fallback methods
                        
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
                
                # Fallback to robot_hat Music or pyttsx3
                if ROBOT_HAT_AVAILABLE and isinstance(self.tts, Music):
                    # Robot HAT Music class doesn't have text-to-speech capability
                    # We need to use OpenAI TTS to generate audio files first
                    self.logger.warn('Robot HAT Music class cannot do text-to-speech directly. Using OpenAI TTS fallback.')
                else:
                    # Use pyttsx3 as fallback
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
        except Exception as e:
            self.logger.error(f'Failed to speak text: {e}')
    
    def play_audio_file(self, file_path):
        """
        Play an audio file with proper mutex handling.
        
        Args:
            file_path: Path to the audio file
        """
        if self.simulation_mode:
            # In simulation mode, just log the file path
            self.logger.debug(f'Simulation: Playing audio file: {file_path}')
            return
        
        # In hardware mode, play the audio file with mutex protection
        try:
            with self.hardware_mutex:
                self.logger.debug(f'Playing audio file: {file_path}')
                
                # Open the audio file
                wf = wave.open(file_path, 'rb')
                
                # Get audio file parameters
                channels = wf.getnchannels()
                sample_width = wf.getsampwidth()
                sample_rate = wf.getframerate()
                
                # Create a stream
                stream = self.audio.open(
                    format=self.audio.get_format_from_width(sample_width),
                    channels=channels,
                    rate=sample_rate,
                    output=True
                )
                
                # Read and play the audio data
                data = wf.readframes(self.chunk_size)
                while data:
                    stream.write(data)
                    data = wf.readframes(self.chunk_size)
                
                # Clean up
                stream.stop_stream()
                stream.close()
                wf.close()
        except Exception as e:
            self.logger.error(f'Failed to play audio file: {e}')
    
    def record_audio(self, duration=5.0, file_path=None):
        """
        Record audio with proper mutex handling.
        
        Args:
            duration: Duration to record (seconds)
            file_path: Path to save the audio file (optional)
        
        Returns:
            Audio data (and saves to file if file_path is provided)
        """
        if self.simulation_mode:
            # In simulation mode, create an empty file and log the duration
            self.logger.debug(f'Simulation: Recording audio for {duration} seconds')
            time.sleep(duration)  # Simulate recording time
            
            # In simulation mode, create an empty audio file if path is provided
            if file_path:
                try:
                    # Create a simple WAV file with silence
                    sample_rate = 16000
                    channels = 1
                    sample_width = 2  # 16-bit
                    
                    # Create 1 second of silence
                    silence_data = b'\x00' * (sample_rate * channels * sample_width)
                    
                    wf = wave.open(file_path, 'wb')
                    wf.setnchannels(channels)
                    wf.setsampwidth(sample_width)
                    wf.setframerate(sample_rate)
                    wf.writeframes(silence_data)
                    wf.close()
                    self.logger.debug(f'Saved simulated audio to {file_path}')
                except Exception as e:
                    self.logger.error(f'Failed to create simulated audio file: {e}')
            
            return None
        
        # In hardware mode, record audio with mutex protection
        try:
            with self.hardware_mutex:
                self.logger.debug(f'Recording audio for {duration} seconds')
                
                # Create a stream
                stream = self.audio.open(
                    format=self.format,
                    channels=self.channels,
                    rate=self.sample_rate,
                    input=True,
                    frames_per_buffer=self.chunk_size
                )
                
                # Record audio
                frames = []
                for i in range(0, int(self.sample_rate / self.chunk_size * duration)):
                    data = stream.read(self.chunk_size)
                    frames.append(data)
                
                # Clean up
                stream.stop_stream()
                stream.close()
                
                # Save to file if requested
                if file_path:
                    wf = wave.open(file_path, 'wb')
                    wf.setnchannels(self.channels)
                    wf.setsampwidth(self.audio.get_sample_size(self.format))
                    wf.setframerate(self.sample_rate)
                    wf.writeframes(b''.join(frames))
                    wf.close()
                    self.logger.debug(f'Saved audio to {file_path}')
                
                return frames
        except Exception as e:
            self.logger.error(f'Failed to record audio: {e}')
            
            # If hardware recording fails but a file path was provided, create a silent file
            if file_path:
                try:
                    # Create a simple WAV file with silence
                    sample_rate = 16000
                    channels = 1
                    sample_width = 2  # 16-bit
                    
                    # Create 1 second of silence
                    silence_data = b'\x00' * (sample_rate * channels * sample_width)
                    
                    wf = wave.open(file_path, 'wb')
                    wf.setnchannels(channels)
                    wf.setsampwidth(sample_width)
                    wf.setframerate(sample_rate)
                    wf.writeframes(silence_data)
                    wf.close()
                    self.logger.debug(f'Saved fallback silent audio to {file_path}')
                except Exception as e2:
                    self.logger.error(f'Failed to create fallback audio file: {e2}')
            
            return None
    
    def set_speech_recognition_parameters(self, energy_threshold=None, pause_threshold=None, dynamic_energy=None):
        """
        Set speech recognition parameters with proper mutex handling.
        
        Args:
            energy_threshold: Energy level threshold for considering an audio frame as speech
            pause_threshold: Seconds of non-speaking audio before a phrase is considered complete
            dynamic_energy: Whether to dynamically adjust the energy threshold based on ambient noise
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
        except Exception as e:
            self.logger.error(f'Failed to set speech recognition parameters: {e}')
    
    # def listen_and_stt(self, timeout=10.0, phrase_time_limit=10.0, language=None):
    #     """
    #     Listen for speech and transcribe it using OpenAI Whisper API in one step.
        
    #     Args:
    #         timeout: Maximum time to wait for speech (seconds)
    #         phrase_time_limit: Maximum time for a phrase (seconds)
    #         language: Language code (defaults to self.language if None)
        
    #     Returns:
    #         Transcribed text or empty string if failed
    #     """
    #     # Listen for speech
    #     audio = self.listen_for_speech(timeout, phrase_time_limit)
        
    #     # Transcribe speech if audio was captured
    #     if audio:
    #         result = self.stt(audio, language)
    #         if result:
    #             return result
    #         else:
    #             # Fall back to recognize_speech if STT fails
    #             self.logger.warn('STT failed, falling back to recognize_speech')
    #             return self.recognize_speech(audio, language)
    #     else:
    #         return ""
    
    # def listen_and_recognize(self, timeout=10.0, phrase_time_limit=10.0, language=None, use_online=True, api='auto'):
    #     """
    #     Listen for speech and recognize it in one step.
        
    #     Args:
    #         timeout: Maximum time to wait for speech (seconds)
    #         phrase_time_limit: Maximum time for a phrase (seconds)
    #         language: Language code (defaults to self.language if None)
    #         use_online: Whether to use online recognition
    #         api: API to use for online recognition ('openai', 'whisper-local', 'google', 'auto', etc.)
        
    #     Returns:
    #         Recognized text
    #     """
    #     # Listen for speech
    #     audio = self.listen_for_speech(timeout, phrase_time_limit)
        
    #     # Recognize speech if audio was captured
    #     if audio:
    #         return self.recognize_speech(audio, language, use_online, api)
    #     else:
    #         return ""
    
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
                # Clean up TTS
                if ROBOT_HAT_AVAILABLE and isinstance(self.tts, Music):
                    try:
                        # Robot HAT Music cleanup
                        self.tts.music_stop()
                    except Exception as e:
                        self.logger.error(f'Failed to clean up Robot HAT Music: {e}')
                elif hasattr(self.tts, 'stop'):
                    try:
                        # pyttsx3 cleanup
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


# Simple test code
def main(args=None):
    """
    Main function for testing the audio hardware interface.
    
    This function initializes ROS2, creates a node and audio hardware interface,
    and runs a series of tests to verify the audio hardware functionality.
    """
    # Check if audio libraries are available
    if not AUDIO_LIBS_AVAILABLE:
        print("Audio libraries are not available")
        print("Please install the required packages to use the audio hardware interface:")
        print("  pip install SpeechRecognition pyaudio")
        if not ROBOT_HAT_AVAILABLE:
            print("  pip install pyttsx3")  # Fallback TTS
        print("For local Whisper support:")
        print("  pip install openai-whisper")
        print("For OpenAI Whisper API support (recommended):")
        print("  pip install openai python-dotenv")
        print("  Add OPENAI_API_KEY to your .env file")
        return
    
    # Check if python-dotenv is installed
    try:
        from dotenv import load_dotenv
    except ImportError:
        print("python-dotenv is not installed")
        print("Please install it to load environment variables from .env file:")
        print("  pip install python-dotenv")
        return
    
    # Check for ffmpeg (needed for mp3 to wav conversion)
    ffmpeg_available = False
    try:
        import subprocess
        subprocess.run(["ffmpeg", "-version"], stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=True)
        ffmpeg_available = True
        print("ffmpeg is available for audio format conversion")
    except (subprocess.SubprocessError, FileNotFoundError):
        print("ffmpeg is not installed or not in PATH")
        print("For better audio handling with OpenAI TTS, install ffmpeg:")
        print("  sudo apt-get install ffmpeg  # On Debian/Ubuntu")
        print("  brew install ffmpeg  # On macOS with Homebrew")
    
    # Check for pygame (fallback for mp3 playback)
    pygame_available = False
    try:
        import pygame
        pygame_available = True
        print("pygame is available for direct mp3 playback")
    except ImportError:
        print("pygame is not installed")
        print("For direct mp3 playback with OpenAI TTS, install pygame:")
        print("  pip install pygame")
    
    # Initialize ROS2
    rclpy.init(args=args)
    
    # Create a node
    node = Node('audio_hardware_interface_test')
    
    # Create the audio hardware interface
    hw = AudioHardwareInterface(node)
    
    try:
        # Test the audio hardware interface
        node.get_logger().info('Testing audio hardware interface...')
        
        # Log environment configuration
        node.get_logger().info('Environment configuration:')
        if hw.openai_api_key:
            node.get_logger().info('  OPENAI_API_KEY: Set (available for TTS and STT)')
        else:
            node.get_logger().info('  OPENAI_API_KEY: Not set (required for OpenAI Whisper API)')
        node.get_logger().info(f'  SPEECH_RECOGNITION_LANGUAGE: {hw.language}')
        node.get_logger().info(f'  SPEECH_RECOGNITION_ENERGY_THRESHOLD: {hw.recognizer.energy_threshold}')
        node.get_logger().info(f'  SPEECH_SYNTHESIS_VOICE: {hw.tts_voice}')
        node.get_logger().info(f'  WHISPER_MODEL: {hw.whisper_model} (used for local Whisper)')
        
        # Test 1: Speech synthesis
        node.get_logger().info('Test 1: Speech synthesis')
        if OPENAI_AVAILABLE and hw.openai_api_key:
            node.get_logger().info('Using OpenAI TTS (Whisper/Onyx) as primary option')
            hw.speak_text("Hello, I am Nevil. Testing OpenAI TTS with Onyx voice.", voice="onyx")
        else:
            node.get_logger().info('OpenAI API not available, using fallback TTS')
            hw.speak_text("Hello, I am Nevil. Testing audio hardware interface.")
        
        time.sleep(1.0)
        
        # Test 2: Speech recognition
        node.get_logger().info('Test 2: Speech recognition')
        node.get_logger().info('Please say something...')
        audio = hw.listen_for_speech(timeout=5.0)
        
        # Test 3: Speech-to-text conversion
        node.get_logger().info('Test 3: Speech recognition with auto API selection')
        text = ""
        if audio:
            # Determine which API to use based on availability
            if OPENAI_AVAILABLE and hw.openai_api_key:
                node.get_logger().info('Using OpenAI Whisper API for speech recognition (primary option)')
                text = hw.recognize_speech(audio, api='openai')
                if not text:
                    node.get_logger().info('OpenAI Whisper API failed, falling back to local methods')
                    if WHISPER_AVAILABLE:
                        node.get_logger().info('Used local Whisper as fallback STT method')
                        text = hw.recognize_speech(audio, api='whisper-local')
                    else:
                        node.get_logger().info('Used Google/Sphinx as fallback STT method')
                        text = hw.recognize_speech(audio)
            elif WHISPER_AVAILABLE:
                node.get_logger().info('Using local Whisper for speech recognition (primary fallback)')
                text = hw.recognize_speech(audio, api='whisper-local')
            else:
                node.get_logger().info('Using Google/Sphinx speech recognition (secondary fallback)')
                text = hw.recognize_speech(audio)
            
            node.get_logger().info(f'Recognized: {text}')
        
        # Test 4: Response with TTS
        node.get_logger().info('Test 4: Repeat what was heard')
        if text:
            # Use Onyx voice for the response if OpenAI TTS is available
            if OPENAI_AVAILABLE and hw.openai_api_key:
                node.get_logger().info('Using OpenAI TTS (Onyx) for response')
                hw.speak_text(f"You said: {text}", voice="onyx")
            else:
                hw.speak_text(f"You said: {text}")
        else:
            hw.speak_text("I didn't hear anything.")
        
        # Test 5: Audio recording
        node.get_logger().info('Test 5: Audio recording')
        node.get_logger().info('Recording for 3 seconds...')
        recording_path = os.path.join(os.getcwd(), "test_recording.wav")
        hw.record_audio(duration=3.0, file_path=recording_path)
        node.get_logger().info(f'Recording saved to {recording_path}')
        
        # Test 6: Audio playback
        node.get_logger().info('Test 6: Audio playback')
        node.get_logger().info('Playing back the recording...')
        hw.play_audio_file(recording_path)
        
        node.get_logger().info('All tests completed')
        
        # Print summary
        node.get_logger().info('Test Summary:')
        node.get_logger().info('- TTS: ' + ('OpenAI Whisper/Onyx (primary)' if OPENAI_AVAILABLE and hw.openai_api_key else
                                          'Robot HAT TTS' if ROBOT_HAT_AVAILABLE else
                                          'pyttsx3 (fallback)'))
        
        # Audio format conversion summary
        if OPENAI_AVAILABLE and hw.openai_api_key:
            try:
                import subprocess
                subprocess.run(["ffmpeg", "-version"], stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=True)
                node.get_logger().info('- Audio Format: ffmpeg available for mp3 to wav conversion')
            except (subprocess.SubprocessError, FileNotFoundError):
                try:
                    import pygame
                    node.get_logger().info('- Audio Format: pygame available for direct mp3 playback')
                except ImportError:
                    node.get_logger().info('- Audio Format: No mp3 support, TTS may be limited')
        
        node.get_logger().info('- STT: ' + ('OpenAI Whisper API (primary)' if OPENAI_AVAILABLE and hw.openai_api_key else
                                          'Local Whisper (fallback)' if WHISPER_AVAILABLE else
                                          'Google/Sphinx (fallback)'))
    except KeyboardInterrupt:
        node.get_logger().info('Test interrupted')
    finally:
        # Clean up
        hw.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()