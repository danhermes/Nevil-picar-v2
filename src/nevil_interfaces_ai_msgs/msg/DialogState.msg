# Dialog state message for Nevil-picar v2.0

# Header with timestamp and frame_id
std_msgs/Header header

# Context ID
string context_id

# Current dialog state
string state                 # 'idle', 'listening', 'processing', 'speaking', 'waiting'

# Current dialog mode
string mode                  # 'command', 'conversation', 'query', 'notification'

# Dialog history (JSON array of utterances)
string dialog_history

# Environment context (JSON object with relevant environment information)
string environment_context

# User information (if available)
string user_id
string user_name

# Active intents and entities (JSON object)
string active_intents

# Last command/response IDs
string last_command_id
string last_response_id

# Dialog metrics
uint32 turn_count            # Number of turns in the current dialog
float32 confidence           # Overall confidence in understanding (0.0 to 1.0)