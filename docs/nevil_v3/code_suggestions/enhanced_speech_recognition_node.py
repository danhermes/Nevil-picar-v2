# nodes/speech_recognition/speech_recognition_node.py - Enhanced

from nevil_framework.base_node import NevilNode
from audio.audio_input import AudioInput
from audio.wake_word_detector import WakeWordDetector, WakeWordConfig
import time
import threading
import os

class SpeechRecognitionNode(NevilNode):
    """Enhanced speech recognition node with wake word detection"""

    def __init__(self):
        super().__init__("speech_recognition")

        # v1.0 audio input (preserved exactly)
        self.audio_input = AudioInput()

        # Wake word detection (new addition)
        wake_config = WakeWordConfig(
            keyword="nevil",
            sensitivity=float(os.getenv("WAKE_WORD_SENSITIVITY", "0.7")),
            access_key=os.getenv("PICOVOICE_ACCESS_KEY")
        )
        self.wake_word_detector = WakeWordDetector(wake_config)

        # State management
        self.listening_mode = "wake_word"  # "wake_word", "active_listening", "disabled"
        self.active_listening_timeout = 10.0  # seconds
        self.active_listening_start = 0

    def initialize(self):
        """Initialize with wake word detection"""
        try:
            # Initialize v1.0 audio input
            self.audio_input = AudioInput()

            # Initialize wake word detection
            if not self.wake_word_detector.initialize():
                self.logger.warning("Wake word detection failed, using continuous listening")
                self.listening_mode = "active_listening"

            # Subscribe to system messages
            self.subscribe("system_mode", self.on_system_mode_change)
            self.subscribe("speaking_status", self.on_speaking_status_change)

            # Start wake word detection
            if self.listening_mode == "wake_word":
                self.wake_word_detector.start_listening(self._on_wake_word_detected)
                self.logger.info("Wake word detection active - say 'Nevil' to activate")

            self.logger.info("Speech recognition initialized with wake word support")

        except Exception as e:
            self.logger.error(f"Failed to initialize speech recognition: {e}")
            raise

    def main_loop(self):
        """Main processing loop with wake word awareness"""
        try:
            if self.listening_mode == "active_listening":
                # Check for active listening timeout
                if time.time() - self.active_listening_start > self.active_listening_timeout:
                    self._return_to_wake_word_mode()
                    return

                # Use v1.0 speech recognition (preserved exactly)
                audio_data = self.audio_input.listen_for_speech(timeout=1.0)

                if audio_data:
                    # Process in separate thread to avoid blocking
                    threading.Thread(
                        target=self._process_audio,
                        args=(audio_data,),
                        daemon=True
                    ).start()

            # Brief pause
            time.sleep(0.1)

        except Exception as e:
            self.logger.error(f"Error in speech recognition main loop: {e}")

    def _on_wake_word_detected(self):
        """Handle wake word detection - switch to active listening"""
        self.logger.info("Wake word detected - activating speech recognition")

        # Switch to active listening mode
        self.listening_mode = "active_listening"
        self.active_listening_start = time.time()

        # Publish system mode change
        self.publish("system_mode", {
            "mode": "listening",
            "source": "wake_word",
            "timestamp": time.time()
        })

        # Publish wake word detection event
        self.publish("wake_word_detected", {
            "keyword": "nevil",
            "confidence": 0.8,  # Would come from detector
            "timestamp": time.time()
        })

    def _return_to_wake_word_mode(self):
        """Return to wake word detection mode"""
        self.logger.info("Returning to wake word detection mode")

        self.listening_mode = "wake_word"

        # Restart wake word detection if not already running
        if not self.wake_word_detector.listening:
            self.wake_word_detector.start_listening(self._on_wake_word_detected)

        # Publish system mode change
        self.publish("system_mode", {
            "mode": "idle",
            "source": "timeout",
            "timestamp": time.time()
        })

    def _process_audio(self, audio_data):
        """Process audio data using v1.0 approach"""
        try:
            # Use exact v1.0 recognition approach
            text = self.audio_input.recognize_speech(audio_data)

            if text:
                self.logger.info(f"Recognized speech: {text}")

                # Publish voice command
                self.publish("voice_command", {
                    "text": text,
                    "confidence": 0.8,  # Would come from recognizer
                    "timestamp": time.time(),
                    "language": "en-US"
                })

                # Reset active listening timer
                self.active_listening_start = time.time()

        except Exception as e:
            self.logger.error(f"Error processing audio: {e}")

    def on_system_mode_change(self, message):
        """Handle system mode changes"""
        mode = message.data.get("mode")
        source = message.data.get("source", "unknown")

        if mode == "idle" and source != "timeout":
            # External request to go idle
            self._return_to_wake_word_mode()
        elif mode == "listening" and source != "wake_word":
            # External request to start listening
            self.listening_mode = "active_listening"
            self.active_listening_start = time.time()

    def on_speaking_status_change(self, message):
        """Handle speaking status changes"""
        is_speaking = message.data.get("speaking", False)

        if is_speaking:
            # Temporarily disable wake word detection during speech output
            self.wake_word_detector.stop_listening()
        else:
            # Re-enable wake word detection after speech
            if self.listening_mode == "wake_word":
                self.wake_word_detector.start_listening(self._on_wake_word_detected)

    def get_health_status(self):
        """Get node health status"""
        base_health = super().get_health_status()

        # Add wake word specific health info
        wake_stats = self.wake_word_detector.get_stats()

        base_health.update({
            "listening_mode": self.listening_mode,
            "wake_word_detections": wake_stats.get("detections_count", 0),
            "wake_word_listening": wake_stats.get("is_listening", False),
            "last_detection": wake_stats.get("last_detection_time", 0)
        })

        return base_health

    def cleanup(self):
        """Enhanced cleanup with wake word detector"""
        # Clean up wake word detector
        if self.wake_word_detector:
            self.wake_word_detector.cleanup()

        # Clean up v1.0 audio input
        if self.audio_input:
            self.audio_input.cleanup()

        self.logger.info("Speech recognition cleaned up")