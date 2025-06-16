# Voice command message for Nevil-picar v2.0

# Header with timestamp and frame_id
std_msgs/Header header

# Recognized text from speech
string recognized_text

# Audio data (if needed for further processing)
# This is optional and may be empty if only text is needed
nevil_interfaces_ai_msgs/Audio audio_data

# Confidence level of recognition (0.0 to 1.0)
float32 confidence

# Command ID for tracking
string command_id

# Context information
string context_id            # ID of the conversation context

# Speaker identification (if available)
string speaker_id