#!/bin/bash

# Modified wrapper script for ROS2 commands that sources the local setup.bash
# This script uses the installed package

echo "***** NEVIL INSTALLED SCRIPT *****"
echo "Current directory: $(pwd)"
echo "Script location: $0"

# Source the ROS2 setup directly
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    echo "Sourced ROS2 Humble setup from /opt/ros/humble/setup.bash"
elif [ -f "/home/dan/ros2_humble/install/setup.bash" ]; then
    source /home/dan/ros2_humble/install/setup.bash
    echo "Sourced ROS2 Humble setup from /home/dan/ros2_humble/install/setup.bash"
else
    echo "Error: ROS2 setup.bash not found. Please install ROS2 or modify this script."
    exit 1
fi

# Source the local setup
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
    echo "Sourced local setup from install/setup.bash"
else
    echo "Error: Local setup.bash not found. Please build the workspace first."
    exit 1
fi

echo "AMENT_PREFIX_PATH: $AMENT_PREFIX_PATH"
echo "Command to execute: ros2 launch nevil_interfaces_ai speech_interface.launch.py"

# Execute the ROS2 command
ros2 launch nevil_interfaces_ai speech_interface.launch.py