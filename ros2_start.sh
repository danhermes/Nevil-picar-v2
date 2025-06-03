#!/bin/bash

# Source the workspace
source /home/dan/ros2_humble/install/setup.bash

# Define the ros2 function to replace the missing command
ros2() {
  # Check if the first argument is a known command
  case "$1" in
    "topic")
      python3 -m ros2topic "${@:2}"
      ;;
    "service")
      python3 -m ros2service "${@:2}"
      ;;
    "node")
      python3 -m ros2node "${@:2}"
      ;;
    "param")
      python3 -m ros2param "${@:2}"
      ;;
    "run")
      python3 -m ros2run "${@:2}"
      ;;
    *)
      echo "Unknown or unsupported ros2 command: $1"
      echo "Supported commands: topic, service, node, param, run"
      return 1
      ;;
  esac
}

# Export the function so it's available in the shell
export -f ros2

echo "ROS2 wrapper is now active. You can use: ros2 topic, ros2 service, ros2 node, ros2 param, ros2 run"


