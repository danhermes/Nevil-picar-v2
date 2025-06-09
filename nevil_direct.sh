#!/bin/bash

# Modified wrapper script for ROS2 commands that doesn't require sourcing setup.bash
# This script uses the direct_speech_interface.launch.py file

echo "***** NEVIL DIRECT SCRIPT *****"
echo "Current directory: $(pwd)"
echo "Script location: $0"

# Source the ROS2 setup directly
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
elif [ -f "/home/dan/ros2_humble/install/setup.bash" ]; then
    source /home/dan/ros2_humble/install/setup.bash
else
    echo "Error: ROS2 setup.bash not found. Please install ROS2 or modify this script."
    exit 1
fi

# Set up environment variables
export PYTHONPATH=$(pwd):$PYTHONPATH

echo "PYTHONPATH: $PYTHONPATH"
echo "Command to execute: ros2 launch nevil_interfaces_ai direct_speech_interface.launch.py"

# Execute the ROS2 command
ros2 launch src/nevil_interfaces_ai/launch/direct_speech_interface.launch.py