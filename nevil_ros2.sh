#!/bin/bash

# Combined wrapper script for Nevil ROS2 environment
# This script sets up both FastRTPS middleware and the Python virtual environment

# Source ROS2 environment
if [ -f /home/dan/ros2_humble/install/setup.bash ]; then
    source /home/dan/ros2_humble/install/setup.bash
elif [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
else
    echo "Error: Could not find ROS2 Humble setup.bash"
    exit 1
fi

# Set FastRTPS as the middleware
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Activate the Python virtual environment
if [ -d "nevil_venv" ]; then
    source nevil_venv/bin/activate
else
    echo "Warning: Python virtual environment not found at nevil_venv/"
fi

# Print environment information
echo "Nevil ROS2 Environment"
echo "======================"
echo "ROS2 Distribution: Humble"
echo "RMW Implementation: $RMW_IMPLEMENTATION"
echo "Python Virtual Env: nevil_venv"
echo "ROS2 Command: ros2 $@"
echo "======================"

# Run the ROS2 command with all arguments
ros2 "$@"

# Store the exit code
EXIT_CODE=$?

# Deactivate the virtual environment
if [ -d "nevil_venv" ]; then
    deactivate
fi

# Return the exit code of the ROS2 command
exit $EXIT_CODE
