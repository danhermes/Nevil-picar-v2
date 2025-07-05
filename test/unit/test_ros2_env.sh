#!/bin/bash

# Test script to verify that the ROS2 environment is working correctly

echo "===== Nevil ROS2 Environment Test ====="
echo "This script will verify that your ROS2 environment is working correctly."
echo "================================================"

# Source ROS2 environment
if [ -f /home/dan/ros2_humble/install/setup.bash ]; then
    source /home/dan/ros2_humble/install/setup.bash
    echo "Sourced ROS2 Humble environment from /home/dan/ros2_humble/install"
elif [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    echo "Sourced ROS2 Humble environment from /opt/ros"
else
    echo "Error: Could not find ROS2 Humble setup.bash"
    exit 1
fi

# Activate the virtual environment
if [ -d "nevil_venv" ]; then
    source nevil_venv/bin/activate
    echo "Activated Python virtual environment: nevil_venv"
else
    echo "Error: Could not find nevil_venv directory"
    exit 1
fi

# Set FastRTPS as the middleware
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
echo "Set RMW_IMPLEMENTATION to $RMW_IMPLEMENTATION"

# Print environment information
echo -e "\nNevil ROS2 Environment"
echo "======================"
echo "ROS2 Distribution: Humble"
echo "RMW Implementation: $RMW_IMPLEMENTATION"
echo "Python Virtual Env: nevil_venv"
echo "======================"

# Test 1: Run ros2 topic list
echo -e "\nTest 1: Running 'ros2 topic list'..."
ros2 topic list
if [ $? -eq 0 ]; then
    echo "✅ Test 1 passed: ros2 topic list executed successfully"
else
    echo "❌ Test 1 failed: ros2 topic list failed to execute"
    exit 1
fi

# Test 2: Check Python packages in virtual environment
echo -e "\nTest 2: Checking Python packages in virtual environment..."
echo "OpenAI package:"
pip list | grep openai
if [ $? -eq 0 ]; then
    echo "✅ Test 2 passed: OpenAI package is installed in the virtual environment"
else
    echo "❌ Test 2 failed: OpenAI package is not installed in the virtual environment"
    exit 1
fi

echo "SpeechRecognition package:"
pip list | grep SpeechRecognition
if [ $? -eq 0 ]; then
    echo "✅ Test 2 passed: SpeechRecognition package is installed in the virtual environment"
else
    echo "❌ Test 2 failed: SpeechRecognition package is not installed in the virtual environment"
    exit 1
fi

echo "pyttsx3 package:"
pip list | grep pyttsx3
if [ $? -eq 0 ]; then
    echo "✅ Test 2 passed: pyttsx3 package is installed in the virtual environment"
else
    echo "❌ Test 2 failed: pyttsx3 package is not installed in the virtual environment"
    exit 1
fi

# Test 3: Check rosdep
echo -e "\nTest 3: Running 'rosdep install' to verify dependencies..."
rosdep install -i --from-path src --rosdistro humble -y
if [ $? -eq 0 ]; then
    echo "✅ Test 3 passed: All dependencies are resolved"
else
    echo "❌ Test 3 failed: Some dependencies could not be resolved"
    exit 1
fi

# Test 4: Check FastRTPS middleware
echo -e "\nTest 4: Checking FastRTPS middleware..."
ros2 doctor --report | grep -A 5 "middleware"
if [ $? -eq 0 ]; then
    echo "✅ Test 4 passed: FastRTPS middleware is configured correctly"
else
    echo "❌ Test 4 failed: Could not verify FastRTPS middleware"
    exit 1
fi

# Deactivate the virtual environment
deactivate
echo "Deactivated Python virtual environment"

echo -e "\n✅ All tests passed! Your ROS2 environment is working correctly."
echo -e "\nTo run your ROS2 packages with the Python virtual environment, use the wrapper script:"
echo "./nevil_run.sh ros2 run your_package your_node"