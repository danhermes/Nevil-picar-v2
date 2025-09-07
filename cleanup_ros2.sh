#!/bin/bash

# ROS2 Cleanup Script
# This script ensures all ROS2 nodes and the daemon are properly cleaned up

echo "🧹 Cleaning up ROS2 environment..."

# Kill any remaining ROS2 processes
echo "Stopping any running ROS2 nodes..."
pkill -f "ros2 run"
pkill -f "ros2 launch"

# Specifically target audio and AI interface nodes that tend to stick around
echo "Stopping audio and AI interface nodes..."
pkill -f "speech_synthesis_node"
pkill -f "speech_recognition_node"
pkill -f "ai_interface_node"
pkill -f "nevil_interfaces_ai"

# Force kill any stubborn audio processes
echo "Force killing stubborn audio processes..."
pkill -9 -f "speech_synthesis_node" 2>/dev/null || true
pkill -9 -f "speech_recognition_node" 2>/dev/null || true
pkill -9 -f "ai_interface_node" 2>/dev/null || true

# Wait and check if any are still running
sleep 2
if pgrep -f "speech_synthesis_node" > /dev/null; then
    echo "⚠️  speech_synthesis_node still running, finding and killing specific PIDs..."
    for pid in $(pgrep -f "speech_synthesis_node"); do
        echo "Killing speech_synthesis_node PID: $pid"
        kill -9 $pid 2>/dev/null || true
    done
fi

if pgrep -f "speech_recognition_node" > /dev/null; then
    echo "⚠️  speech_recognition_node still running, finding and killing specific PIDs..."
    for pid in $(pgrep -f "speech_recognition_node"); do
        echo "Killing speech_recognition_node PID: $pid"
        kill -9 $pid 2>/dev/null || true
    done
fi

# Stop the ROS2 daemon to clear any ghost nodes
echo "Stopping ROS2 daemon..."
ros2 daemon stop 2>/dev/null || true

# Wait a moment for cleanup
sleep 1

# Restart the daemon fresh
echo "Starting fresh ROS2 daemon..."
ros2 daemon start

echo "✅ ROS2 cleanup complete!"
echo "Current nodes:"
ros2 node list