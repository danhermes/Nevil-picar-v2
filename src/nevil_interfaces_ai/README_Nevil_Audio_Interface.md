# Nevil Audio Interface

I've successfully implemented Whisper text-to-speech (Onyx) as a primary option in the AudioHardwareInterface class. To get the complete speech interface working, follow these steps:

## Environment Setup:

Create or update your .env file in the project root with:
OPENAI_API_KEY=your_openai_api_key_here

txt


Verify environment loading:
python3 src/nevil_interfaces_ai/nevil_interfaces_ai/test_env_loading.py

bash


Audio Hardware Testing:

Test microphone and speaker functionality:
python3 src/nevil_interfaces_ai/nevil_interfaces_ai/test_audio_hardware.py

bash


This will verify recording and playback with the new Whisper TTS implementation
Launch Speech Components:

Start speech recognition:
ros2 run nevil_interfaces_ai speech_recognition_node

bash


Start speech synthesis:
ros2 run nevil_interfaces_ai speech_synthesis_node

bash


Start dialog management:
ros2 run nevil_interfaces_ai dialog_manager_node

bash


Testing the Interface:

Monitor recognized speech:
ros2 topic echo /recognized_speech

bash


Test speech synthesis:
ros2 topic pub --once /text_to_speak std_msgs/String "data: Hello, I am Nevil"

bash


You should hear the high-quality Whisper Onyx voice