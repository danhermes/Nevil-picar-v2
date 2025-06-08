# Audio Hardware Interface for Nevil-picar v2.0

This document describes the Audio Hardware Interface for the Nevil-picar v2.0 system, which provides a thread-safe interface to the microphone and speaker hardware.

## Overview

The `AudioHardwareInterface` class provides a unified interface for audio input/output operations on the Nevil-picar v2.0 system. It handles both speech recognition and speech synthesis, with proper mutex handling for real-time performance.

The interface is designed to work with the physical hardware of the PiCar-X robot, using the robot_hat library for hardware access. It also provides a simulation mode for when the hardware is not available.

## Features

- Thread-safe access to microphone and speaker hardware
- Support for both online and offline speech recognition
- Support for both online and offline speech synthesis
- Integration with OpenAI Whisper for improved speech recognition
- Fallback mechanisms for when hardware or libraries are not available
- Simulation mode for development and testing without hardware
- Configuration via environment variables (.env file)

## Dependencies

The interface requires the following dependencies:

- `robot_hat`: For accessing the PiCar-X hardware
- `speech_recognition`: For speech recognition
- `pyaudio`: For audio recording and playback
- `whisper` (optional): For improved offline speech recognition using OpenAI's Whisper model (no API key needed)
- `pyttsx3` (optional): Fallback for speech synthesis when robot_hat TTS is not available

## Configuration

The audio hardware interface can be configured using environment variables in a `.env` file at the root of the project. This allows for easy configuration without modifying the code.

Example `.env` file:

```
# OpenAI API Keys (for language processing, not needed for Whisper)
OPENAI_API_KEY=your_openai_api_key_here

# Speech Recognition Settings
SPEECH_RECOGNITION_LANGUAGE=en
SPEECH_RECOGNITION_ENERGY_THRESHOLD=300
SPEECH_RECOGNITION_PAUSE_THRESHOLD=0.5
SPEECH_RECOGNITION_DYNAMIC_ENERGY=true

# Speech Synthesis Settings
SPEECH_SYNTHESIS_VOICE=onyx
SPEECH_SYNTHESIS_RATE=200
SPEECH_SYNTHESIS_VOLUME=1.0

# Whisper Settings (offline speech-to-text, no API key needed)
WHISPER_MODEL=small  # Options: tiny, base, small, medium, large
```

### Available Configuration Options

| Environment Variable | Description | Default Value |
|----------------------|-------------|---------------|
| `OPENAI_API_KEY` | OpenAI API key for language processing (not needed for Whisper) | None |
| `SPEECH_RECOGNITION_LANGUAGE` | Language code for speech recognition | "en" |
| `SPEECH_RECOGNITION_ENERGY_THRESHOLD` | Energy threshold for speech detection | 300 |
| `SPEECH_RECOGNITION_PAUSE_THRESHOLD` | Pause threshold for speech detection | 0.5 |
| `SPEECH_RECOGNITION_DYNAMIC_ENERGY` | Whether to use dynamic energy threshold | true |
| `SPEECH_SYNTHESIS_VOICE` | Voice ID for speech synthesis | "onyx" |
| `SPEECH_SYNTHESIS_RATE` | Speech rate in words per minute | 200 |
| `SPEECH_SYNTHESIS_VOLUME` | Speech volume (0.0 to 1.0) | 1.0 |
| `WHISPER_MODEL` | Whisper model to use (offline, no API key needed) | "small" |

## Usage

### Initialization

```python
from nevil_interfaces_ai.audio_hardware_interface import AudioHardwareInterface

# Create the interface with a ROS2 node for logging
audio_hw = AudioHardwareInterface(node)

# Or create it without a node
audio_hw = AudioHardwareInterface()
```

### Speech Recognition

```python
# Listen for speech
audio = audio_hw.listen_for_speech(timeout=10.0, phrase_time_limit=10.0)

# Recognize speech
text = audio_hw.recognize_speech(audio, language='en', use_online=True, api='auto')

# Or do both in one step
text = audio_hw.listen_and_recognize(timeout=10.0, language='en', use_online=True)
```

### Speech Synthesis

```python
# Speak text
audio_hw.speak_text("Hello, I am Nevil.", voice="onyx", wait=True)

# Play an audio file
audio_hw.play_audio_file("path/to/audio.wav")
```

### Audio Recording

```python
# Record audio for 5 seconds
audio_data = audio_hw.record_audio(duration=5.0)

# Record audio and save to file
audio_data = audio_hw.record_audio(duration=5.0, file_path="recording.wav")
```

### Configuration

```python
# Set speech recognition parameters
audio_hw.set_speech_recognition_parameters(
    energy_threshold=300,
    pause_threshold=0.5,
    dynamic_energy=True
)

# Set speech synthesis parameters
audio_hw.set_speech_rate(200)
audio_hw.set_speech_volume(1.0)
audio_hw.set_speaker_voice("onyx")
```

### Cleanup

```python
# Clean up resources when done
audio_hw.cleanup()
```

## Implementation Details

The interface uses mutex locks to ensure thread safety when accessing hardware resources. It also provides fallback mechanisms for when hardware or libraries are not available, allowing the system to run in simulation mode.

For speech recognition, the interface uses OpenAI's Whisper model when available, with fallbacks to Google Speech Recognition and Sphinx for offline recognition.

For speech synthesis, the interface uses the robot_hat TTS module when available, with a fallback to pyttsx3 for offline synthesis.

## Error Handling

The interface includes comprehensive error handling to ensure robustness in various scenarios:

- Hardware initialization failures
- Network connectivity issues
- Library availability issues
- Runtime errors

In case of errors, the interface will log appropriate messages and fall back to simulation mode if necessary.

## Integration with ROS2

The interface is designed to work seamlessly with ROS2 nodes, providing logging through the node's logger when available. It is used by the following ROS2 nodes:

- `speech_recognition_node.py`: For converting speech to text
- `speech_synthesis_node.py`: For converting text to speech

## License

This software is part of the Nevil-picar v2.0 project and is licensed under the same terms as the project.