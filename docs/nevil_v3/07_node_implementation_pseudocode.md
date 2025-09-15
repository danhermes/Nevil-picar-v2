
# Nevil v3.0 Node Implementation Pseudocode

## Overview

This document provides detailed pseudocode for the three core nodes in Nevil v3.0: Speech Recognition, Speech Synthesis, and AI Cognition. Each implementation preserves the proven working approaches from v1.0 while integrating with the new **declarative messaging architecture**.

**Key Enhancement**: All nodes now use the `init_messages()` method for automatic messaging setup based on `.messages` configuration files, eliminating manual subscribe/publish calls and significantly simplifying node implementation.

## Detailed Technical Design Specification

### Node Architecture Philosophy

The Nevil v3.0 node implementations are built on a **domain-driven design approach** where each node represents a distinct functional domain with clear boundaries and responsibilities. The architecture emphasizes **behavioral consistency**, **fault tolerance**, and **performance optimization** while maintaining the proven functionality from v1.0.

#### Core Design Principles

1. **Domain Separation**: Each node handles a specific domain (audio input, audio output, AI processing)
2. **State Management**: Explicit state management with clear state transitions
3. **Resource Lifecycle**: Proper resource acquisition, usage, and cleanup

Phase 1:
### Speech Recognition Node Technical Architecture

The Speech Recognition Node implements a **real-time audio processing pipeline** with advanced voice activity detection and confidence-based filtering.

#### Audio Processing Pipeline Architecture

```
Microphone → Audio Buffer → VAD → Speech Detection → Recognition → Confidence Filter → Message Publication
     ↓            ↓           ↓         ↓              ↓              ↓                ↓
Hardware → Ring Buffer → Voice Activity → Segment → OpenAI Whisper → Threshold → Message Bus
```

#### Voice Activity Detection Framework

**Multi-Stage VAD Algorithm**:
1. **Energy-Based Detection**: Initial filtering based on audio energy levels
2. **Spectral Analysis**: Frequency domain analysis for speech characteristics
3. **Temporal Consistency**: Time-based validation to reduce false positives
4. **Adaptive Thresholding**: Dynamic threshold adjustment based on ambient noise

**VAD Configuration Parameters**:
- **Energy Threshold**: Minimum energy level for voice detection (default: 300)
- **Pause Threshold**: Silence duration to end speech segment (default: 0.5s)
- **Minimum Speech Duration**: Minimum duration for valid speech (default: 0.3s)
- **Maximum Speech Duration**: Maximum duration before forced segmentation (default: 10s)

#### Speech Recognition Processing Architecture

**Recognition Pipeline Stages**:
1. **Audio Preprocessing**: Noise reduction and normalization
2. **Feature Extraction**: Conversion to recognition-ready format
3. **API Communication**: Secure communication with OpenAI Whisper
4. **Result Processing**: Confidence estimation and text normalization
5. **Quality Assessment**: Multi-factor quality scoring

**Confidence Estimation Algorithm**:
```python
confidence = base_confidence * length_factor * timing_factor * content_factor
where:
    base_confidence = 0.8  # OpenAI Whisper baseline
    length_factor = optimal_length_curve(word_count)
    timing_factor = processing_time_curve(duration)
    content_factor = content_quality_score(text)
```



### Speech Synthesis Node Technical Architecture

The Speech Synthesis Node implements a **priority-based speech queue** with advanced audio output management and real-time speech control.


#### Text-to-Speech Processing Architecture

**TTS Pipeline Stages**:
1. **Text Preprocessing**: Text normalization and pronunciation optimization
2. **Voice Selection**: Dynamic voice selection based on context and preferences
3. **Audio Generation**: High-quality audio synthesis using OpenAI TTS
4. **Audio Processing**: Post-processing for optimal playback quality
5. **Playback Management**: Synchronized playback with status broadcasting

**Audio Quality Optimization**:
- **Dynamic Range Optimization**: Automatic gain control for consistent volume
- **Frequency Response**: EQ optimization for speaker characteristics
- **Latency Minimization**: Optimized pipeline for minimal speech delay
- **Quality Monitoring**: Real-time monitoring of audio quality metrics

#### Speech Control Framework



### AI Cognition Node Technical Architecture (phase 1)

The AI Cognition Node implements a **context-aware conversation management system** with advanced natural language processing and intelligent response generation.



#### Natural Language Understanding Framework



#### Response Generation Architecture (phase 1)

**Multi-Stage Response Generation**:
1. **Context Analysis**: Analysis of current conversation context
2. **Intent Processing**: Processing of user intent and requirements
3. **Response Planning**: Strategic planning of response content and structure
4. **Content Generation**: AI-powered content generation using OpenAI GPT
5. **Response Optimization**: Post-processing and optimization for delivery





### Inter-Node Communication Architecture (phase 1)

The nodes implement a **sophisticated communication protocol** that ensures reliable, efficient, and secure message exchange.

#### Message Flow Patterns

**Request-Response Pattern**:
```
Node A → Request Message → Message Bus → Node B
Node B → Response Message → Message Bus → Node A
```

**Publish-Subscribe Pattern**:
```
Publisher Node → Topic Message → Message Bus → All Subscribers
```

**Event-Driven Pattern**:
```
Event Source → Event Message → Message Bus → Event Handlers
```

#### Communication Quality Assurance



### Error Handling and Recovery Architecture (phase 1)

Each node implements a **comprehensive error handling framework** that provides graceful degradation and automatic recovery.

#### Error Classification and Response (phase 1)

**Error Severity Levels**:
- **Fatal Errors**: Errors that require node restart or system shutdown
- **Critical Errors**: Errors that significantly impact functionality
- **Warning Errors**: Errors that may impact performance but allow continued operation
- **Informational Errors**: Errors that provide diagnostic information

**Recovery Strategies by Error Type**:
| Error Type | Recovery Strategy | Escalation Path |
|------------|------------------|-----------------|
| Transient Network | Retry with backoff | Service degradation |
| API Rate Limit | Queue and delay | Alternative service |
| Hardware Failure | Fallback mode | Manual intervention |
| Configuration Error | Default values | Configuration reload |





This comprehensive technical design ensures that the Nevil v3.0 node implementations provide robust, high-performance, and maintainable functionality while preserving the proven approaches from v1.0 and integrating seamlessly with the new framework architecture.

## 1. Speech Recognition Node

### 1.1 Core Implementation

```python
# nodes/speech_recognition/speech_recognition_node.py

import time
import threading
import queue
import uuid
from typing import Optional, Dict, Any
from nevil_framework.base_node import NevilNode
from audio.audio_input import AudioInput  # v1.0 proven component

class SpeechRecognitionNode(NevilNode):
    """
    Speech Recognition Node for Nevil v3.0
    
    Converts voice input to text commands using proven v1.0 audio pipeline.
    
    Features:
    - Continuous voice activity detection
    - OpenAI Whisper integration
    - Configurable sensitivity and thresholds
    - Automatic pause during speech synthesis
    - Confidence scoring and filtering
    - Wake word detection
    """
    
    def __init__(self):
        super().__init__("speech_recognition")
        
        # Audio processing components
        self.audio_input = None
        self.listening = False
        self.paused = False
        
        # Processing queues
        self.audio_queue = queue.Queue(maxsize=10)
        self.processing_thread = None
        
        # Configuration
        self.confidence_threshold = 0.5
        self.wake_words = ["nevil", "hey nevil"]
        self.wake_word_mode = False
        
        # Statistics
        self.stats = {
            'commands_recognized': 0,
            'audio_processed': 0,
            'recognition_failures': 0,
            'average_confidence': 0.0,
            'wake_word_detections': 0
        }
    
    def initialize(self):
        """Initialize speech recognition components"""
        try:
            self.logger.info("Initializing speech recognition...")
            
            # Initialize audio input using v1.0 proven approach
            self.audio_input = AudioInput()
            
            # Verify audio hardware
            if not self.audio_input.microphone:
                raise RuntimeError("Microphone not available")
            
            # Note: No manual subscribe() calls needed!
            # All subscriptions are automatically configured via .messages file:
            # - speaking_status -> on_speaking_status_change
            # - system_mode -> on_system_mode_change
            # - audio_config -> on_audio_config_change
            # This happens during init_messages() in __init__
            
            # Start audio processing thread
            self.processing_thread = threading.Thread(
                target=self._audio_processing_loop,
                name=f"{self.node_name}_audio_processor",
                daemon=True
            )
            self.processing_thread.start()
            
            # Start listening
            self.start_listening()
            
            self.logger.info("Speech recognition initialized successfully")
            
        except Exception as e:
            self.logger.error(f"Failed to initialize speech recognition: {e}")
            raise
    
    def main_loop(self):
        """Main processing loop"""
        try:
            if self.listening and not self.paused and self.audio_input:
                # Listen for speech with timeout
                audio_data = self.audio_input.listen_for_speech(
                    timeout=1.0,
                    phrase_time_limit=10.0,
                    adjust_for_ambient_noise=False  # Already adjusted during init
                )
                
                if audio_data:
                    # Add to processing queue
                    try:
                        self.audio_queue.put_nowait(audio_data)
                        self.stats['audio_processed'] += 1
                    except queue.Full:
                        self.logger.warning("Audio processing queue full, dropping audio")
            
            # Brief pause to prevent busy waiting
            time.sleep(0.1)
            
        except Exception as e:
            self.logger.error(f"Error in speech recognition main loop: {e}")
            time.sleep(1.0)
    
    def _audio_processing_loop(self):
        """Background audio processing loop"""
        while not self.shutdown_event.is_set():
            try:
                # Get audio from queue
                try:
                    audio_data = self.audio_queue.get(timeout=1.0)
                except queue.Empty:
                    continue
                
                # Process audio
                self._process_audio_data(audio_data)
                
            except Exception as e:
                self.logger.error(f"Error in audio processing loop: {e}")
                time.sleep(0.5)
    
    def _process_audio_data(self, audio_data):
        """Process audio data and extract text"""
        try:
            start_time = time.time()
            
            # Use v1.0 proven speech recognition
            text = self.audio_input.recognize_speech(
                audio_data,
                language="en-US",
                use_online=True,
                api="openai"
            )
            
            processing_time = time.time() - start_time
            
            if text and text.strip():
                confidence = self._estimate_confidence(text, processing_time)
                
                self.logger.info(f"Recognized: '{text}' (confidence: {confidence:.2f})")
                
                # Check confidence threshold
                if confidence >= self.confidence_threshold:
                    # Check for wake words if in wake word mode
                    if self.wake_word_mode:
                        if self._check_wake_word(text):
                            self.wake_word_mode = False
                            self.stats['wake_word_detections'] += 1
                            self.publish("system_mode", {
                                "mode": "listening",
                                "reason": "wake_word_detected",
                                "timestamp": time.time()
                            })
                            return
                        else:
                            # Not a wake word, ignore
                            return
                    
                    # Publish voice command
                    self._publish_voice_command(text, confidence)
                    
                    # Update statistics
                    self.stats['commands_recognized'] += 1
                    self._update_average_confidence(confidence)
                    
                else:
                    self.logger.debug(f"Low confidence recognition ignored: '{text}' ({confidence:.2f})")
            
            else:
                self.logger.debug("No speech recognized")
                
        except Exception as e:
            self.logger.error(f"Error processing audio data: {e}")
            self.stats['recognition_failures'] += 1
    
    def _publish_voice_command(self, text: str, confidence: float):
        """Publish recognized voice command"""
        command_data = {
            "text": text,
            "confidence": confidence,
            "timestamp": time.time(),
            "node_id": self.node_name,
            "language": "en-US"
        }
        
        self.publish("voice_command", command_data)
        
        # Also publish audio level for UI feedback
        self.publish("audio_level", {
            "level": confidence,
            "timestamp": time.time()
        })
    
    def _estimate_confidence(self, text: str, processing_time: float) -> float:
        """Estimate confidence based on text characteristics and processing time"""
        confidence = 0.8  # Base confidence for OpenAI Whisper
        
        # Adjust based on text length (very short or very long may be less reliable)
        text_length = len(text.split())
        if text_length < 2:
            confidence -= 0.2
        elif text_length > 20:
            confidence -= 0.1
        
        # Adjust based on processing time (very fast may indicate silence)
        if processing_time < 0.5:
            confidence -= 0.3
        elif processing_time > 10.0:
            confidence -= 0.1
        
        # Check for common recognition artifacts
        artifacts = ["um", "uh", "er", "ah"]
        if any(artifact in text.lower() for artifact in artifacts):
            confidence -= 0.1
        
        return max(0.0, min(1.0, confidence))
    
    def _check_wake_word(self, text: str) -> bool:
        """Check if text contains a wake word"""
        text_lower = text.lower()
        return any(wake_word in text_lower for wake_word in self.wake_words)
    
    def _update_average_confidence(self, confidence: float):
        """Update running average confidence"""
        current_avg = self.stats['average_confidence']
        count = self.stats['commands_recognized']
        
        if count == 1:
            self.stats['average_confidence'] = confidence
        else:
            # Running average
            self.stats['average_confidence'] = (current_avg * (count - 1) + confidence) / count
    
    def start_listening(self):
        """Start listening for speech"""
        if not self.listening:
            self.listening = True
            self.logger.info("Started listening for speech")
            
            # Publish listening status
            self.publish("listening_status", {
                "listening": True,
                "reason": "manual_start",
                "timestamp": time.time()
            })
    
    def stop_listening(self):
        """Stop listening for speech"""
        if self.listening:
            self.listening = False
            self.logger.info("Stopped listening for speech")
            
            # Publish listening status
            self.publish("listening_status", {
                "listening": False,
                "reason": "manual_stop",
                "timestamp": time.time()
            })
    
    def pause_listening(self):
        """Temporarily pause listening"""
        self.paused = True
        self.logger.debug("Paused listening")
    
    def resume_listening(self):
        """Resume listening after pause"""
        self.paused = False
        self.logger.debug("Resumed listening")
    
    def on_speaking_status_change(self, message):
        """Handle speaking status changes"""
        try:
            is_speaking = message.data.get("speaking", False)
            
            if is_speaking:
                self.pause_listening()
                self.logger.debug("Paused listening - speech synthesis active")
            else:
                self.resume_listening()
                self.logger.debug("Resumed listening - speech synthesis stopped")
                
        except Exception as e:
            self.logger.error(f"Error handling speaking status change: {e}")
    
    def on_system_mode_change(self, message):
        """Handle system mode changes"""
        try:
            mode = message.data.get("mode")
            
            if mode == "idle":
                self.wake_word_mode = True
                self.logger.info("Entered wake word mode")
            elif mode == "listening":
                self.wake_word_mode = False
                self.start_listening()
            elif mode == "speaking":
                self.pause_listening()
            elif mode == "error":
                self.stop_listening()
                
        except Exception as e:
            self.logger.error(f"Error handling system mode change: {e}")
    
    def on_audio_config_change(self, message):
        """Handle audio configuration changes"""
        try:
            config = message.data
            
            if "energy_threshold" in config:
                if self.audio_input and self.audio_input.recognizer:
                    self.audio_input.recognizer.energy_threshold = config["energy_threshold"]
                    self.logger.info(f"Updated energy threshold: {config['energy_threshold']}")
            
            if "pause_threshold" in config:
                if self.audio_input and self.audio_input.recognizer:
                    self.audio_input.recognizer.pause_threshold = config["pause_threshold"]
                    self.logger.info(f"Updated pause threshold: {config['pause_threshold']}")
            
            if "confidence_threshold" in config:
                self.confidence_threshold = config["confidence_threshold"]
                self.logger.info(f"Updated confidence threshold: {self.confidence_threshold}")
                
        except Exception as e:
            self.logger.error(f"Error handling audio config change: {e}")
    
    def cleanup(self):
        """Cleanup speech recognition resources"""
        try:
            self.stop_listening()
            
            # Clear audio queue
            while not self.audio_queue.empty():
                try:
                    self.audio_queue.get_nowait()
                except queue.Empty:
                    break
            
            # Cleanup audio input
            if self.audio_input:
                self.audio_input.cleanup()
                self.audio_input = None
            
            self.logger.info("Speech recognition cleanup complete")
            
        except Exception as e:
            self.logger.error(f"Error during speech recognition cleanup: {e}")
    
    def get_node_stats(self) -> Dict[str, Any]:
        """Get node-specific statistics"""
        stats = self.stats.copy()
        stats['listening'] = self.listening
        stats['paused'] = self.paused
        stats['wake_word_mode'] = self.wake_word_mode
        stats['queue_size'] = self.audio_queue.qsize()
        
        if self.audio_input:
            stats['microphone_available'] = bool(self.audio_input.microphone)
            stats['simulation_mode'] = self.audio_input.simulation_mode
        
        return stats

# TEST: Speech recognition accurately converts voice to text
# TEST: Wake word detection activates system correctly
# TEST: Audio pausing during speech synthesis works properly
# TEST: Confidence scoring filters low-quality recognition
# TEST: Configuration changes are applied correctly
```

## 2. Speech Synthesis Node

### 2.1 Core Implementation

```python
# nodes/speech_synthesis/speech_synthesis_node.py

import time
import threading
import queue
import uuid
from typing import Optional, Dict, Any
from nevil_framework.base_node import NevilNode
from audio.audio_output import AudioOutput  # v1.0 proven component

class SpeechSynthesisNode(NevilNode):
    """
    Speech Synthesis Node for Nevil v3.0
    
    Converts text to speech using proven v1.0 audio pipeline.
    
    Features:
    - OpenAI TTS integration
    - Priority-based speech queue
    - Voice selection and customization
    - Volume and speed control
    - Interrupt and queue management
    - Speaking status broadcasting
    """
    
    def __init__(self):
        super().__init__("speech_synthesis")
        
        # Audio output component
        self.audio_output = None
        
        # Speech processing
        self.speech_queue = queue.PriorityQueue(maxsize=20)
        self.currently_speaking = False
        self.current_speech_id = None
        self.speech_lock = threading.Lock()
        
        # Configuration
        self.default_voice = "onyx"
        self.default_speed = 1.0
        self.default_volume = 0.8
        self.interrupt_enabled = True
        
        # Statistics
        self.stats = {
            'texts_spoken': 0,
            'speech_failures': 0,
            'total_speech_time': 0.0,
            'queue_overflows': 0,
            'interruptions': 0
        }
    
    def initialize(self):
        """Initialize speech synthesis components"""
        try:
            self.logger.info("Initializing speech synthesis...")
            
            # Initialize audio output using v1.0 proven approach
            self.audio_output = AudioOutput()
            
            # Verify audio hardware
            if self.audio_output.simulation_mode:
                self.logger.warning("Audio output in simulation mode")
            
            # Note: No manual subscribe() calls needed!
            # All subscriptions are automatically configured via .messages file:
            # - text_response -> on_text_response
            # - speech_control -> on_speech_control
            # - audio_config -> on_audio_config_change
            # This happens during init_messages() in __init__
            
            self.logger.info("Speech synthesis initialized successfully")
            
        except Exception as e:
            self.logger.error(f"Failed to initialize speech synthesis: {e}")
            raise
    
    def main_loop(self):
        """Main processing loop"""
        try:
            # Process speech queue
            try:
                # Get next speech item with timeout
                priority, timestamp, speech_data = self.speech_queue.get(timeout=0.1)
                
                # Process the speech
                self._process_speech_request(speech_data)
                
            except queue.Empty:
                pass
            
            # Brief pause if not speaking
            if not self.currently_speaking:
                time.sleep(0.1)
                
        except Exception as e:
            self.logger.error(f"Error in speech synthesis main loop: {e}")
            time.sleep(1.0)
    
    def _process_speech_request(self, speech_data: Dict[str, Any]):
        """Process a speech synthesis request"""
        try:
            with self.speech_lock:
                if self.currently_speaking and self.interrupt_enabled:
                    # Interrupt current speech if higher priority
                    current_priority = getattr(self, '_current_priority', 100)
                    if speech_data.get('priority', 100) < current_priority:
                        self._interrupt_current_speech()
                
                # Set speaking status
                self.currently_speaking = True
                self.current_speech_id = speech_data.get('speech_id', str(uuid.uuid4()))
                self._current_priority = speech_data.get('priority', 100)
                
                # Publish speaking status
                self._publish_speaking_status(True, speech_data)
                
                # Synthesize and play speech
                success = self._synthesize_and_play(speech_data)
                
                if success:
                    self.stats['texts_spoken'] += 1
                else:
                    self.stats['speech_failures'] += 1
                
        except Exception as e:
            self.logger.error(f"Error processing speech request: {e}")
            self.stats['speech_failures'] += 1
        finally:
            # Clear speaking status
            with self.speech_lock:
                self.currently_speaking = False
                self.current_speech_id = None
                self._current_priority = None
                
                # Publish speaking status
                self._publish_speaking_status(False)
    
    def _synthesize_and_play(self, speech_data: Dict[str, Any]) -> bool:
        """Synthesize and play speech"""
        try:
            text = speech_data.get('text', '')
            voice = speech_data.get('voice', self.default_voice)
            speed = speech_data.get('speed', self.default_speed)
            
            if not text.strip():
                return False
            
            self.logger.info(f"Speaking: '{text}' (voice: {voice})")
            
            start_time = time.time()
            
            # Use v1.0 proven audio output
            self.audio_output.speak_text(
                text=text,
                voice=voice,
                wait=True  # Wait for completion
            )
            
            speech_duration = time.time() - start_time
            self.stats['total_speech_time'] += speech_duration
            
            self.logger.info(f"Speech completed in {speech_duration:.2f}s")
            return True
            
        except Exception as e:
            self.logger.error(f"Error synthesizing speech: {e}")
            return False
    
    def _interrupt_current_speech(self):
        """Interrupt currently playing speech"""
        try:
            if self.audio_output:
                # Stop current audio playback
                # Note: This would need to be implemented in audio_output
                self.logger.info("Interrupting current speech")
                self.stats['interruptions'] += 1
                
        except Exception as e:
            self.logger.error(f"Error interrupting speech: {e}")
    
    def _publish_speaking_status(self, speaking: bool, speech_data: Dict = None):
        """Publish speaking status"""
        status_data = {
            "speaking": speaking,
            "timestamp": time.time(),
            "node_id": self.node_name
        }
        
        if speaking and speech_data:
            status_data.update({
                "text": speech_data.get('text', ''),
                "voice": speech_data.get('voice', self.default_voice),
                "speech_id": speech_data.get('speech_id'),
                "priority": speech_data.get('priority', 100)
            })
        
        self.publish("speaking_status", status_data)
    
    def on_text_response(self, message):
        """Handle text response messages"""
        try:
            data = message.data
            text = data.get('text', '')
            
            if not text.strip():
                return
            
            # Create speech request
            speech_data = {
                'speech_id': str(uuid.uuid4()),
                'text': text,
                'voice': data.get('voice', self.default_voice),
                'speed': data.get('speed', self.default_speed),
                'priority': data.get('priority', 100),
                'timestamp': time.time(),
                'source': message.source_node
            }
            
            # Add to queue with priority
            priority = speech_data['priority']
            timestamp = speech_data['timestamp']
            
            try:
                self.speech_queue.put_nowait((priority, timestamp, speech_data))
                self.logger.debug(f"Queued speech: '{text}' (priority: {priority})")
            except queue.Full:
                self.logger.warning("Speech queue full, dropping request")
                self.stats['queue_overflows'] += 1
                
        except Exception as e:
            self.logger.error(f"Error handling text response: {e}")
    
    def on_speech_control(self, message):
        """Handle speech control commands"""
        try:
            command = message.data.get('command')
            
            if command == 'stop':
                self._stop_current_speech()
            elif command == 'pause':
                self._pause_speech()
            elif command == 'resume':
                self._resume_speech()
            elif command == 'clear_queue':
                self._clear_speech_queue()
            elif command == 'set_voice':
                voice = message.data.get('voice', self.default_voice)
                self._set_default_voice(voice)
                
        except Exception as e:
            self.logger.error(f"Error handling speech control: {e}")
    
    def on_audio_config_change(self, message):
        """Handle audio configuration changes"""
        try:
            config = message.data
            
            if "volume" in config:
                self.default_volume = config["volume"]
                if self.audio_output:
                    self.audio_output.set_speech_volume(self.default_volume)
                self.logger.info(f"Updated default volume: {self.default_volume}")
            
            if "voice" in config:
                self.default_voice = config["voice"]
                self.logger.info(f"Updated default voice: {self.default_voice}")
            
            if "speed" in config:
                self.default_speed = config["speed"]
                self.logger.info(f"Updated default speed: {self.default_speed}")
                
        except Exception as e:
            self.logger.error(f"Error handling audio config change: {e}")
    
    def _stop_current_speech(self):
        """Stop currently playing speech"""
        with self.speech_lock:
            if self.currently_speaking:
                self._interrupt_current_speech()
                self.currently_speaking = False
                self._publish_speaking_status(False)
    
    def _pause_speech(self):
        """Pause speech synthesis"""
        # Implementation would depend on audio_output capabilities
        self.logger.info("Speech pause requested")
    
    def _resume_speech(self):
        """Resume speech synthesis"""
        # Implementation would depend on audio_output capabilities
        self.logger.info("Speech resume requested")
    
    def _clear_speech_queue(self):
        """Clear the speech queue"""
        cleared_count = 0
        while not self.speech_queue.empty():
            try:
                self.speech_queue.get_nowait()
                cleared_count += 1
            except queue.Empty:
                break
        
        self.logger.info(f"Cleared {cleared_count} items from speech queue")
    
    def _set_default_voice(self, voice: str):
        """Set the default voice"""
        self.default_voice = voice
        if self.audio_output:
            self.audio_output.set_speaker_voice(voice)
        self.logger.info(f"Set default voice to: {voice}")
    
    def cleanup(self):
        """Cleanup speech synthesis resources"""
        try:
            # Stop current speech
            self._stop_current_speech()
            
            # Clear queue
            self._clear_speech_queue()
            
            # Cleanup audio output
            if self.audio_output:
                self.audio_output.cleanup()
                self.audio_output = None
            
            self.logger.info("Speech synthesis cleanup complete")
            
        except Exception as e:
            self.logger.error(f"Error during speech synthesis cleanup: {e}")
    
    def get_node_stats(self) -> Dict[str, Any]:
        """Get node-specific statistics"""
        stats = self.stats.copy()
        stats['currently_speaking'] = self.currently_speaking
        stats['queue_size'] = self.speech_queue.qsize()
        stats['current_speech_id'] = self.current_speech_id
        stats['default_voice'] = self.default_voice
        stats['default_speed'] = self.default_speed
        stats['default_volume'] = self.default_volume
        
        if self.audio_output:
            stats['simulation_mode'] = self.audio_output.simulation_mode
        
        return stats

# TEST: Speech synthesis produces clear, audible output
# TEST: Priority queue processes high-priority messages first
# TEST: Speech interruption works for urgent messages
# TEST: Voice and speed configuration changes are applied
# TEST: Speaking status is broadcast correctly
```

## 3. AI Cognition Node

### 3.1 Core Implementation

```python
# nodes/ai_cognition/ai_cognition_node.py

import time
import threading
import queue
import json
import uuid
from typing import Optional, Dict, Any, List
from nevil_framework.base_node import NevilNode
from openai import OpenAI

class AICognitionNode(NevilNode):
    """
    AI Cognition Node for Nevil v3.0
    
    Processes voice commands and generates intelligent responses using OpenAI GPT.
    
    Features:
    - OpenAI GPT integration
    - Conversation context management
    - Command classification and routing
    - Response generation and formatting
    - Fallback to local models
    - Context-aware decision making
    """
    
    def __init__(self):
        super().__init__("ai_cognition")
        
        # AI components
        self.openai_client = None
        self.local_model = None  # For offline fallback
        
        # Processing
        self.command_queue = queue.Queue(maxsize=10)
        self.processing_thread = None
        self.context_lock = threading.Lock()
        
        # Conversation management
        self.conversation_history = []
        self.max_context_length = 20  # conversation turns
        self.context_timeout = 3600.0  # 1 hour
        
        # Configuration
        self.model = "gpt-3.5-turbo"
        self.max_tokens = 150
        self.temperature = 0.7
        self.confidence_threshold = 0.6
        
        # Statistics
        self.stats = {
            'commands_processed': 0,
            'responses_generated': 0,
            'api_failures': 0,
            'fallback_uses': 0,
            'average_response_time': 0.0,
            'context_resets': 0
        }
    
    def initialize(self):
        """Initialize AI cognition components"""
        try:
            self.logger.info("Initializing AI cognition...")
            
            # Initialize OpenAI client
            api_key = os.getenv("OPENAI_API_KEY")
            if not api_key:
                raise ValueError("OPENAI_API_KEY environment variable not set")
            
            self.openai_client = OpenAI(api_key=api_key, timeout=30.0)
            
            # Test API connection
            self._test_api_connection()
            
            # Subscribe to voice commands
            self.subscribe("voice_command", self.on_voice_command)
            self.subscribe("text_command", self.on_text_command)
            self.subscribe("system_heartbeat", self.on_system_heartbeat)
            
            # Start command processing thread
            self.processing_thread = threading.Thread(
                target=self._command_processing_loop,
                name=f"{self.node_name}_processor",
                daemon=True
            )
            self.processing_thread.start()
            
            # Initialize conversation context
            self._initialize_conversation_context()
            
            self.logger.info("AI cognition initialized successfully")
            
        except Exception as e:
            self.logger.error(f"Failed to initialize AI cognition: {e}")
            raise
    
    def main_loop(self):
        """Main processing loop"""
        try:
            # Periodic context cleanup
            self._cleanup_old_context()
            
            # Health check for API connection
            if time.time() % 300 < 1:  # Every 5 minutes
                self._health_check_api()
            
            time.sleep(1.0)
            
        except Exception as e:
            self.logger.error(f"Error in AI cognition main loop: {e}")
            time.sleep(5.0)
    
    def _command_processing_loop(self):
        """Background command processing loop"""
        while not self.shutdown_event.is_set():
            try:
                # Get command from queue
                try:
                    command_data = self.command_queue.get(timeout=1.0)
                except queue.Empty:
                    continue
                
                # Process the command
                self._process_command(command_data)
                
            except Exception as e:
                self.logger.error(f"Error in command processing loop: {e}")
                time.sleep(0.5)
    
    def _process_command(self, command_data: Dict[str, Any]):
        """Process a voice or text command"""
        try:
            start_time = time.time()
            
            text = command_data.get('text', '').strip()
            confidence = command_data.get('confidence', 1.0)
            source = command_data.get('source', 'unknown')
            
            if not text:
                return
            
            # Check confidence threshold
            if confidence < self.confidence_threshold:
                self.logger.debug(f"Low confidence command ignored: '{text}' ({confidence:.2f})")
                return
            
            self.logger.info(f"Processing command: '{text}' (confidence: {confidence:.2f})")
            
            # Classify command type
            command_type = self._classify_command(text)
            
            # Generate response
            response = self._generate_response(text, command_type, source)
            
            if response:
                # Publish response
                self._publish_response(response, command_data)
                
                # Update statistics
                processing_time = time.time() - start_time
                self._update_response_time_stats(processing_time)
                self.stats['responses_generated'] += 1
                
                self.logger.info(f"Generated response in {processing_time:.2f}s: '{response[:50]}...'")
            
            self.stats['commands_processed'] += 1
            
        except Exception as e:
            self.logger.error(f"Error processing command: {e}")
            self.stats['api_failures'] += 1
    
    def _classify_command(self, text: str) -> str:
        """Classify the type of command"""
        text_lower = text.lower()
        
        # Question patterns
        question_words = ['what', 'how', 'when', 'where', 'why', 'who', 'which']
        if any(word in text_lower for word in question_words):
            return "question"
        
        # Command patterns
        command_words = ['turn', 'move', 'go', 'stop', 'start', 'play', 'pause']
        if any(word in text_lower for word in command_words):
            return "command"
        
        # Greeting patterns
        greeting_words = ['hello', 'hi', 'hey', 'good morning', 'good afternoon']
        if any(word in text_lower for word in greeting_words):
            return "greeting"
        
        # Default to conversation
        return "conversation"
    
    def _generate_response(self, text: str, command_type: str, source: str) -> Optional[str]:
        """Generate AI response to user input"""
        try:
            # Add user input to conversation history
            with self.context_lock:
                self.conversation_history.append({
                    "role": "user",
                    "content": text,
                    "timestamp": time.time(),
                    "command_type": command_type,
                    "source": source
                })
                
                # Trim context if too long
                if len(self.conversation_history) > self.max_context_length:
                    removed = self.conversation_history.pop(0)
                    self.stats['context_resets'] += 1
            
            # Prepare messages for API
            messages = self._prepare_api_messages()
            
            # Call OpenAI API
            response = self._call_openai_api(messages)
            
            if response:
                # Add response to conversation history
                with self.context_lock:
                    self.conversation_history.append({
                        "role": "assistant",
                        "content": response,
                        "timestamp": time.time()
                    })
                
                return response
            else:
                # Fallback to local model or simple responses
                return self._generate_fallback_response(text, command_type)
                
        except Exception as e:
            self.logger.error(f"Error generating response: {e}")
            return self._generate_fallback_response(text, command_type)
    
    def _prepare_api_messages(self) -> List[Dict[str, str]]:
        """Prepare messages for OpenAI API call"""
        messages = [
            {
                "role": "system",
                "content": "You are Nevil, a helpful and friendly robot companion. "
                          "You can move around, see your environment, and help with various tasks. "
                          "Keep responses concise and conversational. "
                          "If asked to perform actions, acknowledge and describe what you would do."
            }
        ]
        
        # Add recent conversation history
        with self.context_lock:
            for entry in self.conversation_history[-10:]:  # Last 10 exchanges
                if entry["role"] in ["user", "assistant"]:
                    messages.append({
                        "role": entry["role"],
                        "content": entry["content"]
                    })
        
        return messages
    
    def _call_openai_api(self, messages: List[Dict[str, str]]) -> Optional[str]:
        """Call OpenAI API to generate response"""
        try:
            if not self.openai_client:
                return None
            
            response = self.openai_client.chat.completions.create(
                model=self.model,
                messages=messages,
                max_tokens=self.max_tokens,
                temperature=self.temperature,
                timeout=30.0
            )
            
            if response.choices and response.choices[0].message:
                return response.choices[0].message.content.strip()
            
            return None
            
        except Exception as e:
            self.logger.error(f"OpenAI API call failed: {e}")
            self.stats['api_failures'] += 1
            return None
    
    def _generate_fallback_response(self, text: str, command_type: str) -> str:
        """Generate fallback response when API is unavailable"""
        self.stats['fallback_uses'] += 1
        
        fallback_responses = {
            "greeting": [
                "Hello! How can I help you today?",
                "Hi there! What would you like to do?",
                "Hey! I'm here and ready to assist."
            ],
            "question": [
                "That's an interesting question. Let me think about that.",
                "I'd be happy to help, but I need to process that information.",
                "Good question! I'm working on understanding that better."
            ],
            "command": [
                "I understand you'd like me to do something. Let me see what I can do.",
                "I'll try to help with that request.",
                "I'm processing your command."
            ],
            "conversation": [
                "I'm listening and processing what you said.",
                "That's interesting. Tell me more.",
                "I'm here to chat and help however I can."
            ]
        }
        
        import random
        responses = fallback_responses.get(command_type, fallback_responses["conversation"])
        return random.choice(responses)
    
    def _publish_response(self, response: str, original_command: Dict[str, Any]):
        """Publish AI response"""
        response_data = {
            "text": response,
            "voice": "onyx",  # Default voice
            "priority": 100,  # Normal priority
            "timestamp": time.time(),
            "source_command": original_command.get('text', ''),
            "command_type": self._classify_command(original_command.get('text', '')),
            "response_id": str(uuid.uuid4())
        }
        
        self.publish("text_response", response_data)
    
    def _test_api_connection(self):
        """Test OpenAI API connection"""
        try:
            test_response = self.openai_client.chat.completions.create(
                model=self.model,
                messages=[{"role": "user", "content": "Hello"}],
                max_tokens=10,
                timeout=10.0
            )
            self.logger.info("OpenAI API connection test successful")
        except Exception as e:
            self.logger.warning(f"OpenAI API connection test failed: {e}")
    
    def _health_check_api(self):
        """Periodic health check for API"""
        try:
            # Simple API health check
            self._test_api_connection()
        except Exception as e:
            self.logger.warning(f"API health check failed: {e}")
    
    def _initialize_conversation_context(self):
        """Initialize conversation context"""
        with self.context_lock:
            self.conversation_history = [
                {
                    "role": "system",
                    "content": "Conversation started",
                    "timestamp": time.time()
                }
            ]
    
    def _cleanup_old_context(self):
        """Clean up old conversation context"""
        current_time = time.time()
        
        with self.context_lock:
            # Remove entries older than context timeout
            self.conversation_history = [
                entry for entry in self.conversation_history
                if current_time - entry.get('timestamp', 0) < self.context_timeout
            ]
    
    def _update_response_time_stats(self, response_time: float):
        """Update response time statistics"""
        current_avg = self.stats['average_response_time']
        count = self.stats['responses_generated']
        
        if count == 0:
            self.stats['average_response_time'] = response_time
        else:
            # Running average
            self.stats['average_response_time'] = (current_avg * count + response_time) / (count + 1)
    
    def on_voice_command(self, message):
        """Handle voice command messages"""
        try:
            command_data = message.data.copy()
            command_data['source'] = 'voice'
            command_data['message_id'] = message.message_id
            
            # Add to processing queue
            try:
                self.command_queue.put_nowait(command_data)
            except queue.Full:
                self.logger.warning("Command queue full, dropping voice command")
                
        except Exception as e:
            self.logger.error(f"Error handling voice command: {e}")
    
    def on_text_command(self, message):
        """Handle text command messages"""
        try:
            command_data = message.data.copy()
            command_data['source'] = 'text'
            command_data['message_id'] = message.message_id
            
            # Add to processing queue
            try:
                self.command_queue.put_nowait(command_data)
            except queue.Full:
                self.logger.warning("Command queue full, dropping text command")
                
        except Exception as e:
            self.logger.error(f"Error handling text command: {e}")
    
    def on_system_heartbeat(self, message):
        """Handle system heartbeat messages for health monitoring"""
        try:
            # Could use this to monitor other nodes and adjust behavior
            node_name = message.data.get('node_name')
            status = message.data.get('status')
            
            if node_name and status:
                self.logger.debug(f"Heartbeat from {node_name}: {status}")
                
        except Exception as e:
            self.logger.error(f"Error handling system heartbeat: {e}")
    
    def cleanup(self):
        """Cleanup AI cognition resources"""
        try:
            # Clear command queue
            while not self.command_queue.empty():
                try:
                    self.command_queue.get_nowait()
                except queue.Empty:
                    break
            
            # Clear conversation history
            with self.context_lock:
                self.conversation_history.clear()
            
            # Cleanup OpenAI client
            self.openai_client = None
            
            self.logger.info("AI cognition cleanup complete")
            
        except Exception as e:
            self.logger.error(f"Error during AI cognition cleanup: {e}")
    
    def get_node_stats(self) -> Dict[str, Any]:
        """Get node-specific statistics"""
        stats = self.stats.copy()
        stats['queue_size'] = self.command_queue.qsize()
        stats['context_length'] = len(self.conversation_history)
        stats['api_available'] = bool(self.openai_client)
        stats['model'] = self.model
        stats['max_tokens'] = self.max_tokens
        stats['temperature'] = self.temperature
        
        return stats

# TEST: AI cognition generates appropriate responses to various inputs
# TEST: Conversation context is maintained across interactions
# TEST: Fallback responses work when API is unavailable
# TEST: Command classification works for different input types
# TEST: Response timing meets performance requirements
```

## 4. Node Integration Patterns

### 4.1 Inter-Node Communication

```python
# Example of how nodes communicate with each other

# Speech Recognition → AI Cognition
voice_command_message = {
    "topic": "voice_command",
    "data": {
        "text": "What time is it?",
        "confidence": 0.95,
        "timestamp": time.time(),
        "language": "en-US"
    }
}

# AI Cognition → Speech Synthesis
text_response_message = {
    "topic": "text_response",
    "data": {
        "text": "It's currently 3:30 PM.",
        "voice": "onyx",
        "priority": 100,
        "timestamp": time.time()
    }
}

# Speech Synthesis → All Nodes
speaking_status_message = {
    "topic": "speaking_status",
    "data": {
        "speaking": True,
        "text": "It's currently 3:30 PM.",
        "timestamp": time.time()
    }
}
```

### 4.2 Error Handling Patterns

```python
# Common error handling pattern for all nodes

def safe_operation(self, operation_name: str, operation_func, *args, **kwargs):
    """Safely execute an operation with error handling"""
    try:
        self.logger.debug(f"Starting {operation_name}")
        result = operation_func(*args, **kwargs)
        self.logger.debug(f"Completed {operation_name}")
        return result
        
    except Exception as e:
        self.logger.error(f"Error in {operation_name}: {e}")
        
        # Report error to error handler
        error_id = self.error_handler.handle_error(
            node_name=self.node_name,
            error=e,
            context={
                'operation': operation_name,
                'args': str(args)[:100],  # Truncate for logging
                'kwargs': str(kwargs)[:100]
            }
        )
        
        # Return appropriate default or raise
        return self._get_error_fallback(operation_name, e)
```

### 4.3 Configuration Update Patterns

```python
# Pattern for handling configuration updates

def on_config_update(self, message):
    """Handle configuration updates"""
    try:
        config_section = message.data.get('section')
        config_data = message.data.get('data', {})
        
        if config_section == self.node_name:
            # Update node-specific configuration
            for key, value in config_data.items():
                if hasattr(self, key):
                    old_value = getattr(self, key)
                    setattr(self, key, value)
                    self.logger.info(f"Updated {key}: {old_value} → {value}")
                    
                    # Apply configuration change
                    self._apply_config_change(key, value, old_value)
        
    except Exception as e:
        self.logger.error(f"Error handling config update: {e}")

def _apply_config_change(self, key: str, new_value: Any, old_value: Any):
    """Apply a specific configuration change"""
    if key == 'confidence_threshold':
        self.logger.info(f"Confidence threshold updated to {new_value}")
    elif key == 'default_voice':
        if self.audio_output:
            self.audio_output.set_speaker_voice(new_value)
    # Add more configuration handlers as needed
```

## Conclusion

The Nevil v3.0 node implementations provide a solid foundation for the three core system functions while maintaining the proven approaches from v1.0. Key features include:

- **Speech Recognition**: Reliable voice-to-text conversion with wake word detection and confidence filtering
- **Speech Synthesis**: High-quality text-to-speech with priority queuing and voice customization
- **AI Cognition**: Intelligent conversation management with context awareness and fallback capabilities

Each node is designed to be:
- **Autonomous**: Self-contained with clear responsibilities
- **Resilient**: Proper error handling and recovery mechanisms
- **Configurable**: Runtime configuration updates without restart
- **Observable**: Comprehensive statistics and health monitoring
- **Testable**: Clear interfaces and testable components

The implementations preserve the working v1.0 audio pipeline while adding the reliability and observability features needed for a production robotic system.

# TEST: All three nodes can run simultaneously without conflicts
# TEST: Inter-node communication flows work correctly
# TEST: Error conditions are handled gracefully
# TEST: Configuration updates are applied without service interruption
# TEST: Performance meets specified latency requirements