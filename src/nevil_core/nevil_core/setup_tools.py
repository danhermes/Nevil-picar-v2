#!/usr/bin/env python3

import subprocess
import os
import sys

def check_ros2_installation():
    """Check if ROS2 is installed and sourced"""
    try:
        subprocess.run(['ros2', '--help'], check=True, capture_output=True)
        return True
    except (subprocess.CalledProcessError, FileNotFoundError):
        return False

def install_dependencies():
    """Install all required dependencies"""
    commands = [
        "sudo apt update",
        "sudo apt install -y python3-pip python3-rosdep python3-colcon-common-extensions",
        "sudo rosdep init || true",  # || true prevents error if already initialized
        "rosdep update",
        "rosdep install --from-paths src -y --ignore-src"
    ]
    
    for cmd in commands:
        try:
            subprocess.run(cmd.split(), check=True)
        except subprocess.CalledProcessError as e:
            print(f"Error running {cmd}: {e}")
            return False
    return True

def main():
    print("Checking ROS2 installation...")
    if not check_ros2_installation():
        print("ROS2 not found! Please install ROS2 first:")
        print("Visit: https://docs.ros.org/en/humble/Installation.html")
        sys.exit(1)
    
    print("Installing dependencies...")
    if install_dependencies():
        print("Setup completed successfully!")
    else:
        print("Setup failed. Please check the errors above.")

if __name__ == "__main__":
    main() 