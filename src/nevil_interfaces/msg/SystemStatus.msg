# System status message for Nevil-picar v2.0

# Header with timestamp and frame_id
std_msgs/Header header

# System mode
string mode                  # standby, manual, autonomous, learning

# Battery status
float32 battery_level        # 0.0 to 1.0 (percentage/100)
bool low_battery             # True if battery is low

# System health
bool system_ok               # Overall system health
string[] active_nodes        # List of currently active nodes
string[] error_nodes         # List of nodes with errors

# Error information
bool has_errors              # True if system has errors
string[] error_codes         # List of error codes
string[] error_messages      # List of error messages

# Performance metrics
float32 cpu_usage            # 0.0 to 1.0 (percentage/100)
float32 memory_usage         # 0.0 to 1.0 (percentage/100)
float32 disk_usage           # 0.0 to 1.0 (percentage/100)

# System status
bool ok
uint8 mode_code
int32 error_code
string error_message
float32 cpu_temperature
