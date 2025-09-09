#!/usr/bin/env python3

"""
Nevil Audio Play Interface for Nevil-picar v2.0

This module provides a thread-safe hardware interface for audio playback
on the Nevil-picar v2.0 system, interfacing with the speaker hardware.
Based on the original Nevil v1.0 implementation.

Extracted from AudioHardwareInterface to separate play functionality.
"""

import os
import time
import threading
import uuid
import subprocess
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

class NevilAudioPlay:
    """
    Audio playback interface for Nevil-picar v2.0.
    
    This class provides a thread-safe interface to the speaker hardware,
    with proper mutex handling for real-time performance.
    Based on the original Nevil v1.0 implementation.
    """
    
    # Default audio parameters
    DEFAULT_VOLUME_DB = 6
    DEFAULT_TTS_VOICE = "onyx"
    DEFAULT_SPEECH_RATE = 200  # words per minute
    
    def __init__(self, node=None):
        """
        Initialize the Audio Play Interface.
        
        Args:
            node: ROS2 node for logging (optional)
        """
        self.node = node
        self.logger = get_logger('nevil_audio_play') if node is None else node.get_logger()
        
        # Initialize logging
        self.logger.warning('🔊 Audio Play: Initializing...')
        print("🔊 Audio Play: Initializing...")
        
        # Thread safety
        self.hardware_mutex = threading.Lock()
        
        # Initialize state
        self.simulation_mode = False
        self.speech_loaded = False
        self.tts_file = None
        
        # Configure default audio parameters
        self.volume_db = float(get_env_var('SPEECH_SYNTHESIS_VOLUME', self.DEFAULT_VOLUME_DB))
        self.tts_voice = get_env_var('SPEECH_SYNTHESIS_VOICE', self.DEFAULT_TTS_VOICE)
        self.speech_rate = int(get_env_var('SPEECH_SYNTHESIS_RATE', self.DEFAULT_SPEECH_RATE))
        
        # Load OpenAI API key for TTS services
        self.openai_api_key = get_env_var('OPENAI_API_KEY', None)
        if self.openai_api_key:
            self.logger.info('OpenAI API key loaded from environment (for TTS services)')
            # Set it in the environment for TTS services to use
            os.environ["OPENAI_API_KEY"] = self.openai_api_key
        
        # Initialize hardware components
        self.tts = None
        self.music_player = None
        self.speaker_pin = None
        
        # Check if required libraries are available
        try:
            from openai import OpenAI
            OPENAI_AVAILABLE = True
        except ImportError:
            OPENAI_AVAILABLE = False
            self.logger.warning('OpenAI library not available')
        
        try:
            import robot_hat
            ROBOT_HAT_AVAILABLE = True
        except ImportError:
            ROBOT_HAT_AVAILABLE = False
            self.logger.warning('Robot HAT library not available')
        
        if not OPENAI_AVAILABLE or not self.openai_api_key:
            self.logger.warning('🔊 Audio Play: Running in simulation mode')
            print("🔊 Audio Play: Running in simulation mode")
            self.tts = None
            self.music_player = None
            self.speaker_pin = None
            self.simulation_mode = True
        else:
            try:
                # Initialize TTS - Use OpenAI TTS as primary, robot_hat Music only for playback
                self.logger.info(f'OPENAI_AVAILABLE: {OPENAI_AVAILABLE} self.openai_api_key: {self.openai_api_key[3:][:3]}')
                if OPENAI_AVAILABLE and self.openai_api_key:
                    # Primary TTS: OpenAI TTS
                    self.tts = "openai_tts"  # String marker for OpenAI TTS
                    self.logger.info('Using OpenAI TTS as primary text-to-speech engine')
                    
                    # Initialize robot_hat Music for audio playback if available
                    if ROBOT_HAT_AVAILABLE:
                        try:
                            import robot_hat
                            self.music_player = robot_hat.Music()
                            self.speaker_pin = None  # Not needed for audio playback
                            self.logger.info('Robot HAT Music initialized for audio playback')
                        except Exception as e:
                            self.logger.warning(f'Failed to initialize Robot HAT Music for playback: {e}')
                            self.music_player = None
                            self.speaker_pin = None
                    else:
                        self.music_player = None
                        self.speaker_pin = None
                else:
                    self.logger.warn('No TTS engine available (OpenAI not available or no API key)')
                    self.tts = None
                    self.music_player = None
                    self.speaker_pin = None
                
                self.logger.warning('🔊 Audio Play: Audio play interface initialized')
                print("🔊 Audio Play: Audio play interface initialized")
                
            except Exception as e:
                self.logger.error(f'Failed to initialize audio play hardware: {e}')
                print(f"Failed to initialize audio play hardware: {e}")
                
                # Create mock audio interfaces for simulation if hardware initialization fails
                self.tts = None
                self.music_player = None
                self.speaker_pin = None
                self.simulation_mode = True

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
                self.logger.warning(f'🔊 AUDIO PLAY: Speaking text: {text}')
                print(f"🔊 AUDIO PLAY: Speaking text: {text}")
                
                # Use voice parameter if provided, otherwise use default
                tts_voice = voice if voice else self.tts_voice
                
                # Use OpenAI TTS as primary method
                if self.tts == "openai_tts" and self.openai_api_key:
                    try:
                        self.logger.debug(f'Using OpenAI TTS with voice: {tts_voice}')
                        
                        # Create OpenAI client with timeout
                        from openai import OpenAI
                        client = OpenAI(api_key=self.openai_api_key, timeout=30.0)
                        
                        # Generate speech using OpenAI TTS
                        response = client.audio.speech.create(
                            model="tts-1",
                            voice=tts_voice,
                            input=text,
                            response_format="mp3"
                        )
                        
                        # Save audio to temporary file
                        temp_filename = f"/tmp/tts_{uuid.uuid4()}.mp3"
                        with open(temp_filename, "wb") as f:
                            f.write(response.content)
                        
                        # Play the audio file
                        self._play_audio_file(temp_filename, wait)
                        
                        # Clean up temporary file
                        try:
                            os.remove(temp_filename)
                        except Exception as e:
                            self.logger.warning(f'Failed to remove temporary TTS file: {e}')
                        
                    except Exception as e:
                        self.logger.error(f'OpenAI TTS failed: {e}')
                        # Fallback to robot_hat Music if available
                        if self.music_player:
                            self._fallback_tts(text)
                        else:
                            self.logger.error('No fallback TTS available')
                else:
                    self.logger.error('No working TTS engine available')
                    
        except Exception as e:
            self.logger.error(f'Failed to speak text: {e}')

    def _play_audio_file(self, filename, wait=True):
        """Play an audio file using available audio system."""
        try:
            # Try mpg123 first (most reliable) - let it use the asound.conf configuration
            if self._check_command_available('mpg123'):
                self.logger.warning('🔊 AUDIO PLAY: Starting mpg123 with asound.conf configuration')
                print("🔊 AUDIO PLAY: Starting mpg123 with asound.conf configuration")
                # Use mpg123 without explicit device - it will use the asound.conf default
                # Use mpg123 to decode MP3 to stdout, pipe to aplay with default device
                #aplay -D hw:sndrpihifiberry,0 test.wav
                #out123 -o alsa -a hw:sndrpihifiberry,0 test.mp3
                cmd = f'mpg123 -q --stdout "{filename}" | aplay -D hw:sndrpihifiberry,0'
                # cmd = f'mpg123 -o alsa -a hw:sndrpihifiberry,0 "{filename}"'
                # cmd = ['mpg123', '-o', 'alsa', '-a', 'hw:sndrpihifiberry,0', '-q', filename]
                # cmd = ['mpg123', '-q', filename]
                self.logger.warning(f'🔊 AUDIO PLAY: Executing command: {" ".join(cmd)}')
                print(f"🔊 AUDIO PLAY: Executing command: {' '.join(cmd)}")
                
                if wait:
                    result = subprocess.run(cmd, capture_output=True, text=True)
                    if result.returncode == 0:
                        self.logger.warning('🔊 AUDIO PLAY: mpg123 completed successfully')
                        print("🔊 AUDIO PLAY: mpg123 completed successfully")
                    else:
                        self.logger.error(f'mpg123 failed with return code {result.returncode}: {result.stderr}')
                        # Try explicit ALSA as fallback
                        self.logger.warning('🔊 AUDIO PLAY: Trying mpg123 with explicit ALSA fallback')
                        fallback_cmd = ['mpg123', '-o', 'alsa', '-a', 'default', '-q', filename]
                        fallback_result = subprocess.run(fallback_cmd, capture_output=True, text=True)
                        if fallback_result.returncode != 0:
                            self.logger.error(f'mpg123 ALSA fallback also failed: {fallback_result.stderr}')
                        else:
                            self.logger.warning('🔊 AUDIO PLAY: mpg123 ALSA fallback completed successfully')
                else:
                    subprocess.Popen(cmd)
                return
            
            # Fallback to aplay
            if self._check_command_available('aplay'):
                cmd = ['aplay', filename]
                if wait:
                    subprocess.run(cmd, capture_output=True)
                else:
                    subprocess.Popen(cmd)
                return
            
            # Fallback to robot_hat Music
            if self.music_player:
                self.music_player.music_set_volume(100)
                self.music_player.sound_play(filename)
                if wait:
                    time.sleep(2.0)  # Wait for playback
                return
            
            self.logger.error('No audio playback method available')
            
        except Exception as e:
            self.logger.error(f'Failed to play audio file: {e}')

    def _check_command_available(self, command):
        """Check if a command is available in the system."""
        try:
            subprocess.run(['which', command], capture_output=True, check=True)
            return True
        except subprocess.CalledProcessError:
            return False

    def _fallback_tts(self, text):
        """Fallback TTS using robot_hat Music."""
        try:
            if self.music_player:
                self.music_player.music_set_volume(100)
                # Simple text-to-speech using robot_hat (if available)
                self.logger.warning('Using robot_hat Music fallback TTS')
                # This is a placeholder - robot_hat Music doesn't have built-in TTS
                # In a real implementation, you might use espeak or festival
                self.logger.warning(f'Fallback TTS: {text}')
        except Exception as e:
            self.logger.error(f'Fallback TTS failed: {e}')

    def set_speaker_voice(self, voice):
        """Set the TTS voice."""
        self.tts_voice = voice
        self.logger.debug(f'TTS voice set to: {voice}')

    def set_speech_rate(self, rate):
        """Set the speech rate in words per minute."""
        self.speech_rate = rate
        self.logger.debug(f'Speech rate set to: {rate} WPM')

    def set_speech_volume(self, volume):
        """Set the speech volume (0.0 to 1.0)."""
        self.volume_db = volume
        self.logger.debug(f'Speech volume set to: {volume}')

    def cleanup(self):
        """Clean up audio play resources."""
        try:
            with self.hardware_mutex:
                # Clean up TTS engine
                if self.tts and hasattr(self.tts, 'stop'):
                    try:
                        self.tts.stop()
                        self.logger.info('TTS engine stopped')
                    except Exception as e:
                        self.logger.error(f'Failed to stop TTS engine: {e}')
                
                # Clean up music player
                if self.music_player:
                    try:
                        if hasattr(self.music_player, 'music_stop'):
                            self.music_player.music_stop()
                        self.music_player = None
                        self.logger.info('Music player cleaned up')
                    except Exception as e:
                        self.logger.error(f'Failed to clean up music player: {e}')
                
                # Clean up speaker pin
                if self.speaker_pin:
                    try:
                        self.speaker_pin = None
                        self.logger.info('Speaker pin cleaned up')
                    except Exception as e:
                        self.logger.error(f'Failed to clean up speaker pin: {e}')
                        
        except Exception as e:
            self.logger.error(f'Error during audio play cleanup: {e}')
