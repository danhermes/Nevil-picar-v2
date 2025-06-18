#!/bin/bash

# Build script for hardware bridge integration
echo "🔧 Building Nevil-picar v2.0 with Hardware Bridge Integration..."

# Stop any running system
echo "Stopping any running system..."
pkill -f "ros2 launch"
sleep 2

# Clean and build the system
echo "Building packages..."
colcon build --packages-select nevil_interfaces nevil_realtime nevil_navigation --symlink-install

# Source the workspace
echo "Sourcing workspace..."
source install/setup.bash

echo "✅ Build complete! Hardware bridge integration ready."
echo ""
echo "🚀 To test the hardware bridge:"
echo "   ros2 launch nevil_core nevil_system.launch.py use_sim:=false"
echo ""
echo "🔍 To check hardware bridge service:"
echo "   ros2 service list | grep hardware_command"
echo ""
echo "📊 To monitor hardware bridge:"
echo "   ros2 topic echo /rosout | grep hardware_bridge"