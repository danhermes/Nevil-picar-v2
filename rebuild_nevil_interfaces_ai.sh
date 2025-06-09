#!/bin/bash

# Script to rebuild the nevil_interfaces_ai package

echo "***** REBUILDING NEVIL_INTERFACES_AI *****"
echo "Current directory: $(pwd)"

# Source ROS2 setup
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

# Clean the build directory for the package
echo "Cleaning build directory for nevil_interfaces_ai..."
rm -rf build/nevil_interfaces_ai
rm -rf install/nevil_interfaces_ai

# Build the package with symlink install
echo "Building nevil_interfaces_ai with symlink install..."
colcon build --symlink-install --packages-select nevil_interfaces_ai

# Check if the build was successful
if [ $? -eq 0 ]; then
    echo "Build successful!"
    
    # Source the local setup
    source install/setup.bash
    
    # Check if the launch files were installed
    if [ -d "install/nevil_interfaces_ai/share/nevil_interfaces_ai/launch" ]; then
        echo "Launch files were successfully installed!"
        ls -la install/nevil_interfaces_ai/share/nevil_interfaces_ai/launch
    else
        echo "Error: Launch files were not installed."
        echo "Check the setup.py file and build logs for errors."
    fi
else
    echo "Error: Build failed."
    echo "Check the build logs for errors."
fi

echo "***** REBUILD COMPLETE *****"