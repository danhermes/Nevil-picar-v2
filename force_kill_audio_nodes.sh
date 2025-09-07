#!/bin/bash

# Force kill audio and AI interface nodes that tend to stick around
# This script is more aggressive than the regular cleanup

echo "🔨 Force killing stubborn audio nodes..."

# First try gentle kill
pkill -f "speech_synthesis_node" 2>/dev/null || true
pkill -f "speech_recognition_node" 2>/dev/null || true
pkill -f "ai_interface_node" 2>/dev/null || true
pkill -f "nevil_interfaces_ai" 2>/dev/null || true

# Wait a moment
sleep 2

# Check if any are still running and kill by specific PID
if pgrep -f "speech_synthesis_node" > /dev/null; then
    echo "⚠️  speech_synthesis_node still running, killing by PID..."
    for pid in $(pgrep -f "speech_synthesis_node"); do
        echo "Force killing speech_synthesis_node PID: $pid"
        kill -9 $pid 2>/dev/null || true
    done
fi

if pgrep -f "speech_recognition_node" > /dev/null; then
    echo "⚠️  speech_recognition_node still running, killing by PID..."
    for pid in $(pgrep -f "speech_recognition_node"); do
        echo "Force killing speech_recognition_node PID: $pid"
        kill -9 $pid 2>/dev/null || true
    done
fi

if pgrep -f "ai_interface_node" > /dev/null; then
    echo "⚠️  ai_interface_node still running, killing by PID..."
    for pid in $(pgrep -f "ai_interface_node"); do
        echo "Force killing ai_interface_node PID: $pid"
        kill -9 $pid 2>/dev/null || true
    done
fi

# Stop and restart ROS2 daemon to clear ghost nodes
echo "🔄 Restarting ROS2 daemon to clear ghost nodes..."
ros2 daemon stop 2>/dev/null || true
sleep 1
ros2 daemon start

echo "✅ Force kill complete!"
echo "Current nodes:"
ros2 node list
