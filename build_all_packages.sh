#!/bin/bash

# Build All Packages Script for Nevil-picar v2.0
# This script builds all packages in the Nevil-picar v2.0 project
# and creates symbolic links for executable scripts.

set -e  # Exit on error

echo "Building all packages for Nevil-picar v2.0..."

# Define the packages to build in dependency order
PACKAGES=(
  "nevil_interfaces"
  "nevil_interfaces_ai"
  "nevil_core"
  "nevil_navigation"
  "nevil_realtime"
  "nevil_simulation"
  "nevil_testing"
  "nevil_bringup"
)

# Build each package
for package in "${PACKAGES[@]}"; do
  echo "Building package: $package"
  colcon build --packages-select "$package"
  
  # Check if build was successful
  if [ $? -eq 0 ]; then
    echo "✅ Successfully built $package"
  else
    echo "❌ Failed to build $package"
    exit 1
  fi
done

# Create symbolic links for executable scripts in nevil_bringup
echo "Creating symbolic links for executable scripts..."
if [ -d "install/nevil_bringup/lib/nevil_bringup" ]; then
  cd install/nevil_bringup/lib/nevil_bringup
  
  # Create symbolic links for all Python scripts
  for script in *.py; do
    if [ -f "$script" ]; then
      base_name=$(basename "$script" .py)
      echo "Creating symbolic link for $script -> $base_name"
      ln -sf "$script" "$base_name"
    fi
  done
  
  cd ../../../../
fi

echo "Build completed successfully!"
echo "You can now run: ros2 launch nevil_bringup physical_robot.launch.py"