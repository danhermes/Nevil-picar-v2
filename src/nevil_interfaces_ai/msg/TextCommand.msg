# Text command message for Nevil-picar v2.0

# Header with timestamp and frame_id
std_msgs/Header header

# Command text
string command_text

# Command source
string source                # 'user', 'system', 'api'

# Command type
string command_type          # 'navigation', 'query', 'behavior', 'system'

# Command priority
uint8 priority               # 0-255, higher is more important

# Command ID for tracking
string command_id

# Context information
string context_id            # ID of the conversation context