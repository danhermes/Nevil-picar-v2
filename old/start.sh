#!/bin/bash
set -e  # Exit on any error

echo "Starting comprehensive setup for Nevil..."

# 1. Check for Python
if ! command -v python3 &> /dev/null; then
    echo "Installing Python3..."
    sudo apt update && sudo apt install -y python3 python3-pip
fi

# 2. Clean up any existing ROS2 repository files
echo "Cleaning up old ROS2 repositories..."
sudo rm -f /etc/apt/sources.list.d/ros2*.list
sudo rm -f /etc/apt/sources.list.d/ros2.list

# 3. Add ROS2 apt repository
echo "Adding ROS2 apt repository..."
sudo apt update && sudo apt install -y software-properties-common curl gnupg
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# For Debian Bookworm, we'll use Ubuntu Noble repository (Iron) as it's compatible with Python 3.11
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 4. Update package lists
echo "Updating package lists..."
sudo apt update

# 5. Install ROS2 base
echo "Installing ROS2..."
sudo apt install -y ros-iron-ros-base

# 6. Install colcon and other development tools
echo "Installing colcon and development tools..."
sudo apt install -y python3-colcon-common-extensions python3-rosdep

# 7. Create workspace structure if needed
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WORKSPACE_ROOT="$SCRIPT_DIR"

if [ ! -d "$WORKSPACE_ROOT/src" ]; then
    echo "Creating workspace structure..."
    mkdir -p "$WORKSPACE_ROOT/src"
fi

# 8. Run ROS2 setup script (setup_tools.py)
echo "Running ROS2 setup checks..."
if ! python3 "$WORKSPACE_ROOT/src/nevil_core/nevil_core/setup_tools.py"; then
    echo "ROS2 setup failed. Please check the errors above."
    exit 1
fi

# 9. Build with colcon
echo "Building workspace with colcon..."
cd "$WORKSPACE_ROOT"
colcon build

# 10. Source the workspace
SETUP_FILE="$WORKSPACE_ROOT/install/setup.bash"
if [ -f "$SETUP_FILE" ]; then
    echo "Sourcing the workspace..."
    source "$SETUP_FILE"
    
    # Add to bashrc if not already there
    if ! grep -q "$SETUP_FILE" ~/.bashrc; then
        echo "Adding workspace to ~/.bashrc"
        echo "source $SETUP_FILE" >> ~/.bashrc
    fi
else
    echo "Error: Setup file not found at $SETUP_FILE"
    exit 1
fi

echo "
Setup completed successfully!

Your workspace is now ready to use. A new terminal will automatically source
the workspace, or you can run:
source ~/.bashrc

To rebuild the workspace in the future, run:
cd $WORKSPACE_ROOT && colcon build
" 