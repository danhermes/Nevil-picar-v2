# Audio message for Nevil-picar v2.0
# Based on common audio message formats

std_msgs/Header header

# Raw audio data as a byte array
uint8[] data

# Audio format (e.g., "S16LE" for 16-bit signed little-endian)
string format

# Sample rate in Hz (e.g., 16000, 44100)
uint32 sample_rate

# Number of channels (1 for mono, 2 for stereo)
uint8 channels

# Bytes per sample (e.g., 2 for 16-bit audio)
uint8 step

# Whether the data is big-endian
bool is_bigendian