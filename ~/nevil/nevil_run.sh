#!/bin/bash

# Direct wrapper script for running ROS2 commands with nevil_bringup
# This script avoids sourcing setup files with spaces in paths

# Source the ROS2 setup directly
source /home/dan/ros2_humble/install/setup.bash

# Add the nevil packages to the ROS package path directly
export AMENT_PREFIX_PATH=~/nevil/src/install:$AMENT_PREFIX_PATH
export CMAKE_PREFIX_PATH=~/nevil/src/install:$CMAKE_PREFIX_PATH
export LD_LIBRARY_PATH=~/nevil/src/install/lib:$LD_LIBRARY_PATH
export PATH=~/nevil/src/install/bin:$PATH
export PYTHONPATH=~/nevil/src/install/lib/python3.10/site-packages:$PYTHONPATH

# Print the command being executed
echo "Executing: ros2 $*"

# Execute the ROS2 command
ros2 $*