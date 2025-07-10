#!/bin/bash

# ROS2 Cleanup Script
# This script ensures all ROS2 nodes and the daemon are properly cleaned up

echo "🧹 Cleaning up ROS2 environment..."

# Kill any remaining ROS2 processes
echo "Stopping any running ROS2 nodes..."
pkill -f "ros2 run"
pkill -f "ros2 launch"

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