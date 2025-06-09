# Voice response message for Nevil-picar v2.0

# Header with timestamp and frame_id
std_msgs/Header header

# Text to be synthesized
string text_to_speak

# Audio data (if pre-synthesized)
# This is optional and may be empty if synthesis happens downstream
nevil_interfaces_ai/Audio audio_data

# Voice parameters
string voice_id              # Identifier for voice to use
float32 speaking_rate        # 1.0 = normal, <1.0 = slower, >1.0 = faster
float32 pitch                # 1.0 = normal, <1.0 = lower, >1.0 = higher
float32 volume               # 1.0 = normal, <1.0 = quieter, >1.0 = louder

# Original command ID this is responding to
string command_id

# Context information
string context_id            # ID of the conversation context