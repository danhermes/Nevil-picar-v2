# Text response message for Nevil-picar v2.0

# Header with timestamp and frame_id
std_msgs/Header header

# Response text
string response_text

# Response type
string response_type         # 'answer', 'confirmation', 'error', 'status'

# Response status
uint8 status                 # 0 = success, non-zero = error code

# Original command ID this is responding to
string command_id

# Context information
string context_id            # ID of the conversation context

# Additional data (JSON string)
string data