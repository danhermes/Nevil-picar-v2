import warnings
warnings.filterwarnings("ignore", category=RuntimeWarning, module="ALSA")

from helpers.openai_helper import OpenAiHelper
from helpers.action_helper import actions_dict, move_forward_this_way as forward, move_backward_this_way as backward, turn_left_in_place, turn_right_in_place, turn_left, turn_right, stop, shake_head, nod, wave_hands, resist, act_cute, rub_hands, think, keep_think, twist_body, celebrate, depressed, honk, start_engine
from helpers.utils import *
from robot_hat import TTS
import readline # optimize keyboard input, only need to import
import speech_recognition as sr
import os
import sys
from dotenv import load_dotenv
from time import sleep
import numpy as np
import re
from helpers.automatic import Automatic

# Load environment variables from .env file
load_dotenv()

sys.path.append(os.path.abspath(os.path.dirname(__file__)))
from picarlibs.picarx import Picarx
from robot_hat import Music, Pin
import time
import threading
import random
from helpers.load_RAG_files import LoadRAGfiles

class PiCar:
    def __init__(self):
        self.force_quit = False  # Add force quit flag
        self.last_speech_time = 0  # Add this to track when we last spoke
        try:
            # First try to cleanup any existing resources
            from robot_hat import reset_mcu
            reset_mcu()
            time.sleep(0.1)
            
            self.init_system()
            self.init_input_mode()
            self.init_openai()
            self.init_car()
            self.init_camera()
            self.init_speech()
            self.init_sound_effects()
            self.init_threads()
            self.init_topics()
            self.init_movement()
            self.init_auto() 
        except Exception as e:
            print(f"Initialization error: {e}")
            raise

    def init_system(self):
        os.popen("pinctrl set 20 op dh")  # enable robot_hat speaker switch
        self.current_path = os.path.dirname(os.path.abspath(__file__))
        os.chdir(self.current_path)
        self.tts = TTS()
        self.RAGgeneral =""
        self.RAGtext =""

    def init_input_mode(self):
        self.input_mode = 'voice'
        self.with_img = True
        args = sys.argv[1:]
        if '--keyboard' in args:
            self.input_mode = 'keyboard'
        if '--no-img' in args:
            self.with_img = False

    def init_openai(self):
        self.openai_helper = OpenAiHelper(
            os.getenv("OPENAI_API_KEY"), 
            os.getenv("OPENAI_ASSISTANT_ID"), 
            'picarx'
        )
        self.LANGUAGE = "en"
        self.VOLUME_DB = 3
        self.TTS_VOICE = 'onyx'

    def init_car(self):
        try:
            self.my_car = Picarx()
            # Add safety distance attributes
            self.my_car.SafeDistance = 30  # 30cm safe distance
            self.my_car.DangerDistance = 15  # 15cm danger distance
            self.my_car.speed = 30  # Set default speed
            time.sleep(1)
        except Exception as e:
            raise RuntimeError(e)
        self.music = Music()
        self.my_car.music = self.music  # Add this line to attach music to my_car
        self.led = Pin('LED')
        self.DEFAULT_HEAD_TILT = 20

    def init_camera(self):
        gray_print("Initializing camera...")
        if not self.with_img:
            gray_print("Camera disabled, skipping initialization")
            return
            
        try:
            gray_print("Importing camera libraries...")
            from vilib import Vilib
            import cv2
            # Store these for later use
            self.Vilib = Vilib
            self.cv2 = cv2

            gray_print("Starting camera...")
            Vilib.camera_start(vflip=False, hflip=False)
            Vilib.show_fps()
            gray_print("Setting up web display...")
            Vilib.display(local=False, web=True)

            gray_print("Waiting for Flask server to start...")
            while True:
                if Vilib.flask_start:
                    break
                time.sleep(0.01)
            time.sleep(.5)
            gray_print("Camera initialization complete\n")
        except Exception as e:
            gray_print(f"Camera initialization failed: {str(e)}")
            raise

    def init_speech(self):
        self.recognizer = sr.Recognizer()
        
        # Configure speech recognition parameters
        self.recognizer.energy_threshold = 300      # Minimum audio energy level required to detect speech. Lower values (50-4000 range) increase sensitivity to quiet speech but may pick up more background noise
        self.recognizer.dynamic_energy_adjustment_damping = 0.1  # Controls adaptation rate of energy threshold to ambient noise (0-1 range). 0 means instant adaptation, 1 means no adaptation. Lower values better for varying noise environments
        self.recognizer.dynamic_energy_ratio = 1.2  # How much louder speech must be vs ambient noise. At 1.2, speech needs to be 20% louder than background. Higher values reduce false detections but require clearer speech
        self.recognizer.pause_threshold = .5        # Duration of silence in seconds needed to mark end of phrase. 0.5s balances responsiveness with natural speech pauses
        self.recognizer.operation_timeout = 18      # Maximum seconds to wait for speech input before timeout. Balances giving users enough time while preventing indefinite waits
        self.phrase_threshold = 0.5  # minimum seconds of speaking audio before we consider the speaking audio a phrase - values below this are ignored (for filtering out clicks and pops)
        self.non_speaking_duration = 0.5  # seconds of non-speaking audio to keep on both sides of the recording

        # Configure audio with default device - don't try to be clever
        import pyaudio
        self.audio = pyaudio.PyAudio()
        self.audio_device = None  # Use system default
        self.chunk_size = 32768
        
        # Speech handler variables
        self.speech_loaded = False
        self.speech_lock = threading.Lock()
        self.tts_file = None

    def init_sound_effects(self):
        try:
            self.music.pygame.mixer.init(frequency=44100, size=-16, channels=2)
        except Exception as e:
            print(f"Mixer initialization error: {e}")

    def init_threads(self):
        # Action thread variables
        self.action_status = 'standby'
        self.led_status = 'standby'
        self.last_action_status = 'standby'
        self.last_led_status = 'standby'
        self.LED_DOUBLE_BLINK_INTERVAL = 1.2
        self.LED_BLINK_INTERVAL = 0.1
        self.actions_to_be_done = []
        self.action_lock = threading.Lock()

        # Create and start threads
        self.speak_thread = threading.Thread(target=self.speech_handler)
        self.speak_thread.daemon = False  # Change to non-daemon
        self.speak_thread.start()
        
        self.action_thread = threading.Thread(target=self.action_handler)
        self.action_thread.daemon = False  # Change to non-daemon
        self.action_thread.start()

    def init_topics(self):
        self.RAGgeneral = LoadRAGfiles().get_files_from_Dropbox(file_type="general")

    def init_movement(self):
        self.ACTIONS = actions_dict

    def init_auto(self):
        """Initialize automatic mode settings"""
        self.auto = Automatic(self)  # Initialize auto mode
        self.auto_enabled = False

    def play_audio_data(self, audio_data):
        """Play AudioData object and clean up temp files"""
        # Convert AudioData to wav file format that pygame can play
        temp_wav = "./tts/temp_recording.wav"
        with open(temp_wav, "wb") as f:
            f.write(audio_data.get_wav_data())  # Convert AudioData to WAV format
        temp_wav_vol = "./tts/temp_recording_vol.wav"
        status = sox_volume(temp_wav, temp_wav_vol, self.VOLUME_DB)
        
        # Play the temporary wav file
        self.music.music_play(temp_wav_vol)
        while self.music.pygame.mixer.music.get_busy():
            time.sleep(0.1)
        self.music.music_stop()
        
        # Clean up temporary files
        try:
            os.remove(temp_wav)
            os.remove(temp_wav_vol)
        except:
            pass

    def handle_STT(self, audio):
        """Handle speech-to-text conversion"""
        try:
            if audio is None:
                print("No audio data received")
                return None
            
            if check_audio_energy(audio):
                transcript = self.openai_helper.stt(audio, language=self.LANGUAGE)
                print("Transcript:", transcript)
                return transcript
            else:
                print("Silence detected: skipping transcription.")
                return None
            
        except Exception as e:
            print(f"Error in STT: {e}")
            return None

    def listen(self):
        """Listen for audio input and return transcript"""
        with self.action_lock:
            self.action_status = 'standby'

        gray_print("Listening ...")
        self.my_car.set_cam_tilt_angle(self.DEFAULT_HEAD_TILT) #TODO later make this a listen action in handler
        listen_start = time.time()
        _stderr_back = redirect_error_2_null()
        
        try:
            with sr.Microphone(device_index=self.audio_device, 
                              chunk_size=self.chunk_size,
                              sample_rate=44100) as source:
                cancel_redirect_error(_stderr_back)
                self.recognizer.adjust_for_ambient_noise(source)
                try:  
                    audio = self.recognizer.listen(source, timeout=10, phrase_time_limit=10)
                except sr.WaitTimeoutError:
                    print("Timeout detected.")
                    return None, time.time() - listen_start
            
            gray_print(f"Listen time: {time.time() - listen_start:.3f}s")
            
            return audio, time.time() - listen_start
            
        except Exception as e:
            print(f"Error in listen: {e}")
            return None, time.time() - listen_start

    def speech_handler(self):
        while True:
            with self.action_lock:
                if self.action_status == 'shutdown':
                    gray_print("Speech thread shutting down...")
                    break
            
            with self.speech_lock:
                _isloaded = self.speech_loaded

            if _isloaded:             
                gray_print(f"Playing TTS file: {self.tts_file}")
                play_audio_file(self.music, self.tts_file)
                with self.speech_lock:
                    self.speech_loaded = False
            
            time.sleep(0.05)

    def handle_led_status(self, state, last_led_time):
        """Handle LED behavior based on current state"""
        if self.led_status != self.last_led_status:
            last_led_time = 0
            self.last_led_status = self.led_status

        if state == 'standby':
            if time.time() - last_led_time > self.LED_DOUBLE_BLINK_INTERVAL:
                self.led.off()
                self.led.on()
                sleep(.1)
                self.led.off()
                sleep(.1)
                self.led.on()
                sleep(.1)
                self.led.off()
                return time.time()
        elif state == 'think':
            if time.time() - last_led_time > self.LED_BLINK_INTERVAL:
                self.led.off()
                sleep(self.LED_BLINK_INTERVAL)
                self.led.on()
                sleep(self.LED_BLINK_INTERVAL)
                return time.time()
        elif state == 'actions':
            self.led.on()
        
        return last_led_time

    def parse_action(self, response):
        # Handle sleep command
        if response.startswith("sleep"):
            try:
                _, duration = response.split()
                return "sleep", [float(duration)]
            except (ValueError, IndexError):
                print(f"Failed to parse sleep command: {response}")
                return None, None

        # Handle movement commands
        parts = response.lower().split()
        if len(parts) >= 2:
            # Handle forward movement
            if parts[0] == "forward":
                try:
                    distance = int(parts[1])
                    speed = int(parts[2]) if len(parts) > 2 else None
                    gray_print(f"Parsed forward movement: distance={distance}cm" + 
                             (f", speed={speed}" if speed else " with default speed"))
                    return "forward", [distance, speed] if speed else [distance]
                except (IndexError, ValueError):
                    gray_print(f"Failed to parse forward parameters from: {parts}")
                    return None, None
            
            # Handle backward movement
            elif parts[0] == "backward":
                try:
                    distance = int(parts[1])
                    speed = int(parts[2]) if len(parts) > 2 else None
                    gray_print(f"Parsed backward movement: distance={distance}cm" + 
                             (f", speed={speed}" if speed else " with default speed"))
                    return "backward", [distance, speed] if speed else [distance]
                except (IndexError, ValueError):
                    gray_print(f"Failed to parse backward parameters from: {parts}")
                    return None, None
        
        # Handle other actions
        return response, None

    def parse_gpt_response(self, response):
        """Parse GPT response for actions, speech, and mood changes"""
        try:
            if isinstance(response, dict):
                actions = list(response.get('actions', ['stop']))
                answer = response.get('answer', '')
                # Add this to handle mood from JSON response
                if 'mood' in response and self.auto:
                    new_mood = response['mood']
                    old_mood = self.auto.current_mood_name
                    if self.auto.set_mood(new_mood):
                        gray_print(f"[mood] Changed from {old_mood} to {new_mood}")
                        gray_print(f"[mood] New traits: {self.auto.get_mood_param_str()}")
            else:
                actions = []
                answer = str(response)

            # Look for mood change indicators
            mood_match = re.search(r'\[mood: (\w+)\]', answer)
            if mood_match:
                new_mood = mood_match.group(1)
                # Remove mood indicator from answer
                answer = re.sub(r'\[mood: \w+\]', '', answer).strip()
                # Apply mood change
                if self.auto and new_mood:
                    if self.auto.set_mood(new_mood):
                        print(f"[system] Mood changed to: {new_mood}")

            return actions, answer

        except Exception as e:
            print(f"Error parsing GPT response: {e}")
            return [], ''

    def action_handler(self):
        last_led_time = time.time()

        while True:
            with self.action_lock:
                _state = self.action_status
            
            if _state == 'shutdown':
                gray_print("Action thread shutting down...")
                break
            
            # led handling
            self.led_status = _state
            last_led_time = self.handle_led_status(_state, last_led_time)

            # actions
            if _state == 'standby':
                if self.last_action_status != 'standby':
                    gray_print("Entering standby state")
                self.last_action_status = 'standby'
            
            elif _state == 'think':
                if self.last_action_status != 'think':
                    gray_print("Entering think state")
                    self.last_action_status = 'think'
                    self.ACTIONS["keep think"](self.my_car)

            elif _state == 'actions':
                if self.last_action_status != 'actions':
                    gray_print("Entering actions state")
                self.last_action_status = 'actions'
                with self.action_lock:
                    gray_print(f'actions: {self.actions_to_be_done}')
                    _actions = self.actions_to_be_done
                for _action in _actions:
                    try:
                        gray_print(f'action2: {_action}')
                        action, params = self.parse_action(_action)
                        if action == "sleep":
                            time.sleep(params[0])  # Sleep duration is first parameter
                        elif action in self.ACTIONS:
                            if params:
                                self.ACTIONS[action](self.my_car, *params)
                            else:
                                self.ACTIONS[action](self.my_car)
                    except Exception as e:
                        print(f'action error: {e}')
                    time.sleep(0.5)

                gray_print("Actions complete, setting status to actions_done")
                with self.action_lock:
                    self.action_status = 'actions_done'

            time.sleep(0.01)

    def handle_transcript(self, transcript):
        """Handle the transcript from voice input"""
        # Handle no input cases
        if transcript is None:
            if not self.speech_loaded and self.action_status == 'standby':
                print("\n[system] No input detected, running automatic behaviors...")
                self.auto_enabled = True
                self.auto.run_idle_loop(cycles=self.auto.get_cycle_count())
                return True  # Continue main loop
        
        # Check for wake word if in auto mode
        if self.auto_enabled:
            if transcript and "nevil" in transcript.lower():
                print("[system] Wake word detected")
                self.auto_enabled = False
                self.actions_to_be_done = []
                self.my_car.reset()
                with self.action_lock:
                    self.action_status = 'standby'
            else:
                print("\n[system] No wake word, running automatic behaviors...")
                self.auto.run_idle_loop(cycles=self.auto.get_cycle_count())
                return True  # Continue main loop
        
        # Handle shutdown command
        if transcript and "shut down" in transcript.lower():
            gray_print("Shutting down ...")
            raise KeyboardInterrupt()
        
        return False  # Continue with normal processing

    def handle_RAG(self, transcript):
        """Load relevant RAG files based on transcript content"""
        self.RAGtext = ""
        
        # Check for fitness-related content
        fitness_terms = [
            "gym", "fit", "pump", "The Y", "YMCA", "Tai Chi", "walk", "exercise", 
            "motion", "movement", "skateboarding", "dance", "cardio", "strength", 
            "endurance", "flexibility", "balance", "core", "stretching", "yoga", 
            "pilates"
        ]
        if bool(re.search('|'.join(map(re.escape, fitness_terms)), transcript)):
            try:
                gray_print("Getting fitness schedule")    
                self.RAGtext += LoadRAGfiles().get_files_from_Dropbox(file_type="fitness")
            except Exception as e:  
                gray_print(f"Error getting fitness schedule: {e}")
        
        # Check for work-related content
        work_terms = [
            "work", "job", "project", "task", "deadline", "meeting", "customer", 
            "company", "client", "writing", "search", "contract", "proposal", 
            "email", "consulting", "consult", "consultant", "Lexicon", "books", 
            "book", "artwork", "art", "publishers", "galleries", "planning", 
            "event", "talk", "calendar", "raging", "coding", "outreach"
        ]
        if bool(re.search('|'.join(map(re.escape, work_terms)), transcript)):
            try:
                gray_print("Getting work schedule")    
                self.RAGtext += LoadRAGfiles().get_files_from_Dropbox(file_type="work")
            except Exception as e:  
                gray_print(f"Error getting work schedule: {e}")
        
        return self.RAGtext

    def handle_TTS_generation(self, message):
        """Generate TTS file from message"""
        tts_start = time.time()
        try:
            _tts_status = False
            if message != '':
                raw_file, processed_file = generate_tts_filename(volume_db=self.VOLUME_DB)
                
                _tts_status = self.openai_helper.text_to_speech(
                    message, raw_file, self.TTS_VOICE, response_format='wav'
                )
            
            if _tts_status:
                _tts_status = sox_volume(raw_file, processed_file, self.VOLUME_DB)
                os.remove(raw_file)
                self.tts_file = processed_file
                with self.speech_lock:
                    self.speech_loaded = True
                
            gray_print(f'TTS generation time: {time.time() - tts_start:.3f}s')
            return _tts_status
        
        except Exception as e:
            print(f'TTS generation error: {e}')
            return False

    def handle_GPT(self, transcript, rag_content):
        """Handle GPT interaction and response processing"""
        gpt_start = time.time()

        with self.action_lock:
            self.action_status = 'think'

        # Make GPT call
        if self.with_img:
            img_path = './img_input.jpg'
            cv2_start = time.time()
            self.cv2.imwrite(img_path, self.Vilib.img)
            gray_print(f"Image capture time: {time.time() - cv2_start:.3f}s")
            response = self.openai_helper.dialogue_with_img(
                transcript + "   " + self.RAGgeneral + rag_content, 
                img_path
            )
        else:
            response = self.openai_helper.dialogue(
                transcript + "   " + self.RAGgeneral + "   " + rag_content
            )

        gray_print(f'GPT total time: {time.time() - gpt_start:.3f}s')

        # Parse response
        parse_start = time.time()
        actions, GPT_response = self.parse_gpt_response(response)
        gray_print(f"Parse time: {time.time() - parse_start:.3f}s")

        return actions, GPT_response

    def handle_keyboard(self):
        """Handle keyboard input mode"""
        self.my_car.set_cam_tilt_angle(self.DEFAULT_HEAD_TILT)

        with self.action_lock:
            self.action_status = 'standby'

        transcript = input(f'\033[1;30m{"input: "}\033[0m').encode(sys.stdin.encoding).decode('utf-8')

        # Handle shutdown command
        if transcript.lower() == "shut down":
            gray_print("Shutting down ...")
            raise KeyboardInterrupt()
        
        # Process input like voice
        if self.handle_transcript(transcript):
            return True  # for auto mode
        
        rag_content = self.handle_RAG(transcript)
        
        if not self.handle_GPT(transcript, rag_content):
            return True  # on error
        
        return False

    def handle_actions(self, actions):
        """Queue actions and sounds for execution"""
        try:
            with self.action_lock:
                self.actions_to_be_done = actions
                self.action_status = 'actions'
            return True
        except Exception as e:
            print(f'Action handling error: {e}')
            return False

    def wait_for_completion(self):
        """Wait for speech and actions to complete"""
        wait_start = time.time()
        
        if self.speech_loaded:
            while True:
                with self.speech_lock:
                    if not self.speech_loaded:
                        break
                time.sleep(.01)

        while True:
            with self.action_lock:
                if self.action_status != 'actions':
                    break
            time.sleep(.01)
        
        gray_print(f"Wait time for completion: {time.time() - wait_start:.3f}s")

    def main(self):
        self.my_car.reset()
        self.my_car.set_cam_tilt_angle(self.DEFAULT_HEAD_TILT)

        while True:
            if self.input_mode == 'voice':
                audio, listen_time = self.listen()
                transcript = self.handle_STT(audio)
                
                if self.handle_transcript(transcript):
                    continue  # for auto mode
                
                rag_content = self.handle_RAG(transcript)
                actions, GPT_response = self.handle_GPT(transcript, rag_content)
                
                if not GPT_response:
                    continue  # on error
                    
                if not self.handle_TTS_generation(GPT_response):
                    print("TTS generation failed")
                    continue  # on error
                    
                if not self.handle_actions(actions):
                    continue  # on error
                
                self.wait_for_completion()
                gray_print(f"Total iteration time: {time.time() - listen_time:.3f}s\n")

            elif self.input_mode == 'keyboard':
                if self.handle_keyboard():
                    continue  # for auto mode
            else:
                raise ValueError("Invalid input")

    def cleanup(self):
        """Add this method for cleanup"""
        try:
            # Signal threads to stop
            with self.action_lock:
                self.action_status = 'shutdown'
            
            with self.speech_lock:
                self.speech_loaded = False
            
            # Give threads time to finish
            time.sleep(0.5)
            
            # Stop audio first since it's independent
            if hasattr(self, 'music'):
                try:
                    self.music.music_stop()
                    self.music.pygame.mixer.quit()
                except:
                    pass
                
            # Close PyAudio
            if hasattr(self, 'audio'):
                try:
                    self.audio.terminate()
                except:
                    pass

            # Wait for threads to finish before GPIO cleanup
            gray_print("Waiting for threads to finish...")
            try:
                self.speak_thread.join(timeout=1.0)
                self.action_thread.join(timeout=1.0)
            except KeyboardInterrupt:
                pass  # Continue cleanup even if interrupted

            # Let robot_hat handle GPIO cleanup
            try:
                from robot_hat import reset_mcu
                reset_mcu()
                time.sleep(0.2)
            except:
                pass

            gray_print("Cleanup complete")
            sys.exit(0)  # Use sys.exit instead of os._exit
            
        except Exception as e:
            print(f"Cleanup error: {e}")
            sys.exit(1)

    @staticmethod
    def reset_all():
        """Reset all hardware before starting"""
        try:
            # Kill other nevil.py processes but not our own or the grep
            current_pid = os.getpid()
            os.system("ps aux | grep python | grep -v nevil.py | grep -v grep | grep -v $$ | awk '{print $2}' | xargs -r kill -9")
            time.sleep(1)  # Give time for processes to die
            
            # Reset MCU last
            try:
                from robot_hat import reset_mcu
                reset_mcu()
            except Exception as e:
                print(f"MCU reset error: {e}")
            time.sleep(0.5)
            
        except Exception as e:
            print(f"Reset error: {e}")

    def call_GPT(self, user_prompt, use_image=False):
        """Centralized method for GPT interactions"""
        try:
            if use_image:
                img_path = './img_input.jpg'
                self.cv2.imwrite(img_path, self.Vilib.img)
                response = self.openai_helper.dialogue_with_img(
                    user_prompt + "   " + self.RAGgeneral + self.RAGtext, 
                    img_path
                )
            else:
                response = self.openai_helper.dialogue(
                    user_prompt + "   " + self.RAGgeneral + "   " + self.RAGtext
                )

            return self.parse_gpt_response(response)

        except Exception as e:
            print(f"Error in GPT call: {e}")
            return [], ''

if __name__ == "__main__":
    car = None
    try:
        car = PiCar()
        car.main()
    except KeyboardInterrupt:
        print("\nShutting down gracefully...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        try:
            if car:
                car.cleanup()
        except KeyboardInterrupt:
            print("Forced shutdown")