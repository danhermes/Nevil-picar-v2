# Behavior status message for Nevil-picar v2.0

# Header with timestamp and frame_id
std_msgs/Header header

# Behavior information
string behavior_id           # Unique identifier for the behavior
string behavior_name         # Human-readable name of the behavior
string behavior_category     # Category of behavior (navigation, expression, interaction)

# Execution status
string status                # idle, running, completed, failed
float32 progress             # 0.0 to 1.0 (percentage/100)
float32 duration             # Duration of behavior execution in seconds

# Parameters
string[] param_names         # Names of behavior parameters
string[] param_values        # Values of behavior parameters

# Result information (when completed or failed)
bool success                 # True if behavior completed successfully
string result_code           # Result code
string result_message        # Human-readable result message