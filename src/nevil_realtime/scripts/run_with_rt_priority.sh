#!/bin/bash

# This script runs a ROS2 command with real-time priority
# Usage: run_with_rt_priority.sh <priority> <package> <executable> [args...]

# Source ROS2 environment
source /home/dan/ros2_humble/install/setup.bash

# Add the nevil packages to the ROS package path
export AMENT_PREFIX_PATH="$(pwd)/install:$AMENT_PREFIX_PATH"
export CMAKE_PREFIX_PATH="$(pwd)/install:$CMAKE_PREFIX_PATH"
export LD_LIBRARY_PATH="$(pwd)/install/lib:$LD_LIBRARY_PATH"
export PATH="$(pwd)/install/bin:$PATH"
export PYTHONPATH="$(pwd)/install/lib/python3.10/site-packages:$PYTHONPATH"

# Get the priority, package, and executable
PRIORITY=$1
PACKAGE=$2
EXECUTABLE=$3

# Shift the first three arguments
shift 3

# Run the command with real-time priority
# exec chrt -f $PRIORITY ros2 run $PACKAGE $EXECUTABLE "$@"  # ❌ Requires root & RT kernel
exec ros2 run "$PACKAGE" "$EXECUTABLE" "$@"                  # ✅ Standard user-space launch
