# audio/wake_word_detector.py

import threading
import numpy as np
from typing import Callable, Optional, List
import pvporcupine
import struct
from dataclasses import dataclass

@dataclass
class WakeWordConfig:
    keyword: str = "nevil"
    sensitivity: float = 0.7
    model_path: Optional[str] = None
    keyword_paths: List[str] = None
    access_key: Optional[str] = None  # Picovoice access key

class WakeWordDetector:
    """
    Wake word detection using Picovoice Porcupine.

    Provides always-listening capability with low power consumption
    while preserving exact v1.0 audio pipeline for main recognition.
    """

    def __init__(self, config: WakeWordConfig):
        self.config = config
        self.porcupine = None
        self.audio_stream = None
        self.listening = False
        self.detection_callback = None

        # Threading
        self.detection_thread = None
        self.stop_event = threading.Event()

        # Statistics
        self.detections_count = 0
        self.false_positives_count = 0
        self.last_detection_time = 0

        # Integration with v1.0 audio system
        self.audio_input = None  # Will be set by parent AudioInput class

    def initialize(self) -> bool:
        """Initialize Porcupine wake word detection"""
        try:
            # Use built-in "nevil" model or custom model
            if self.config.keyword_paths:
                keyword_paths = self.config.keyword_paths
            else:
                # Use built-in Porcupine keywords or custom trained model
                keyword_paths = [pvporcupine.KEYWORD_PATHS['computer']]  # Placeholder

            self.porcupine = pvporcupine.create(
                access_key=self.config.access_key,
                keyword_paths=keyword_paths,
                sensitivities=[self.config.sensitivity]
            )

            return True

        except Exception as e:
            print(f"Failed to initialize wake word detection: {e}")
            return False

    def start_listening(self, callback: Callable[[], None]):
        """Start wake word detection"""
        if self.listening:
            return

        self.detection_callback = callback
        self.listening = True
        self.stop_event.clear()

        self.detection_thread = threading.Thread(
            target=self._detection_loop,
            name="WakeWordDetection",
            daemon=True
        )
        self.detection_thread.start()

    def stop_listening(self):
        """Stop wake word detection"""
        if not self.listening:
            return

        self.listening = False
        self.stop_event.set()

        if self.detection_thread:
            self.detection_thread.join(timeout=2.0)

    def _detection_loop(self):
        """Main wake word detection loop"""
        try:
            # Use same audio input as v1.0 system
            import pyaudio

            audio = pyaudio.PyAudio()
            stream = audio.open(
                rate=self.porcupine.sample_rate,
                channels=1,
                format=pyaudio.paInt16,
                input=True,
                input_device_index=1,  # Same USB mic as v1.0
                frames_per_buffer=self.porcupine.frame_length
            )

            while self.listening and not self.stop_event.is_set():
                pcm = stream.read(self.porcupine.frame_length, exception_on_overflow=False)
                pcm = struct.unpack_from("h" * self.porcupine.frame_length, pcm)

                keyword_index = self.porcupine.process(pcm)

                if keyword_index >= 0:
                    self._handle_wake_word_detected()

            stream.close()
            audio.terminate()

        except Exception as e:
            print(f"Error in wake word detection loop: {e}")

    def _handle_wake_word_detected(self):
        """Handle wake word detection"""
        import time
        current_time = time.time()

        # Debounce detections (prevent multiple triggers)
        if current_time - self.last_detection_time < 2.0:
            return

        self.last_detection_time = current_time
        self.detections_count += 1

        print(f"Wake word '{self.config.keyword}' detected!")

        if self.detection_callback:
            try:
                self.detection_callback()
            except Exception as e:
                print(f"Error in wake word callback: {e}")

    def get_stats(self) -> dict:
        """Get wake word detection statistics"""
        return {
            "detections_count": self.detections_count,
            "false_positives_count": self.false_positives_count,
            "last_detection_time": self.last_detection_time,
            "is_listening": self.listening
        }

    def cleanup(self):
        """Cleanup resources"""
        self.stop_listening()
        if self.porcupine:
            self.porcupine.delete()