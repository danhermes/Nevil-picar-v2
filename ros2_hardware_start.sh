#!/bin/bash

# Nevil 2.0 Hardware-Aware ROS2 Launcher
# This script ensures proper permissions for hardware access

set -e  # Exit on any error

echo "🤖 Starting Nevil 2.0 with hardware permissions..."

# Check if running as root or with sudo
if [ "$EUID" -eq 0 ]; then
    echo "✅ Running with root privileges"
elif [ -n "$SUDO_USER" ]; then
    echo "✅ Running with sudo"
else
    echo "❌ Hardware access requires sudo privileges"
    echo "Usage: sudo ./ros2_hardware_start.sh"
    exit 1
fi

# Verify hardware access
echo "🔍 Checking hardware permissions..."

# Check GPIO access
if [ ! -r /dev/gpiomem ] || [ ! -w /dev/gpiomem ]; then
    echo "❌ Cannot access /dev/gpiomem"
    exit 1
fi

# Check I2C access
if [ ! -r /dev/i2c-1 ] || [ ! -w /dev/i2c-1 ]; then
    echo "❌ Cannot access /dev/i2c-1"
    exit 1
fi

# Check config file access
CONFIG_FILE="/opt/picar-x/picar-x.conf"
if [ -f "$CONFIG_FILE" ]; then
    if [ ! -r "$CONFIG_FILE" ] || [ ! -w "$CONFIG_FILE" ]; then
        echo "❌ Cannot access config file: $CONFIG_FILE"
        exit 1
    fi
    echo "✅ Config file accessible: $CONFIG_FILE"
else
    echo "⚠️  Config file not found: $CONFIG_FILE"
    echo "   Creating default config file..."
    mkdir -p /opt/picar-x
    cat > "$CONFIG_FILE" << EOF
# PiCar-X Configuration File
# Servo calibration values
picarx_dir_servo=0
picarx_cam_pan_servo=0
picarx_cam_tilt_servo=0

# Motor calibration values
picarx_dir_motor=[1, 1]

# Sensor calibration values
line_reference=[1000, 1000, 1000]
cliff_reference=[500, 500, 500]
EOF
    chmod 666 "$CONFIG_FILE"
    echo "✅ Created default config file"
fi

echo "✅ Hardware permissions verified"

# Source ROS2 environment
echo "🔧 Setting up ROS2 environment..."
if [ -f "/home/dan/ros2_humble/install/setup.bash" ]; then
    source /home/dan/ros2_humble/install/setup.bash
    echo "✅ ROS2 environment sourced"
else
    echo "❌ ROS2 installation not found"
    exit 1
fi

# Set ROS2 domain
export ROS_DOMAIN_ID=0
echo "✅ ROS_DOMAIN_ID set to $ROS_DOMAIN_ID"

# Preserve environment for sudo execution
export -p > /tmp/ros2_env.sh

# Change to workspace directory
cd /home/dan/Nevil-picar-v2

# Build workspace if needed
echo "🔨 Building workspace..."
if [ ! -d "build" ] || [ ! -d "install" ]; then
    echo "Building ROS2 workspace..."
    colcon build --symlink-install
fi

# Source workspace
source install/setup.bash
echo "✅ Workspace sourced"

# Start the system
echo "🚀 Launching Nevil 2.0 system..."

# Check if launch file exists
LAUNCH_FILE="src/nevil_bringup/launch/nevil_system.launch.py"
if [ -f "$LAUNCH_FILE" ]; then
    ros2 launch nevil_bringup nevil_system.launch.py
else
    echo "⚠️  System launch file not found, starting navigation node directly..."
    ros2 run nevil_navigation navigation_node
fi