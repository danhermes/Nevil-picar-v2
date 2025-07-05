#!/usr/bin/env python3

"""
Direct script to run the dialog_manager_node without relying on ROS2 entry points.
This is a temporary solution until the package can be properly built and installed.
"""

import os
import sys
import rclpy

# Add the src directory to the Python path
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'src'))

# Add the specific directory to the Python path
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'src/nevil_interfaces_ai/nevil_interfaces_ai'))

# Import the dialog_manager_node module directly
try:
    # Try direct import
    import dialog_manager_node
    dialog_manager_main = dialog_manager_node.main
    print("Successfully imported dialog_manager_node directly")
except ImportError as e:
    print(f"Error importing dialog_manager_node: {e}")
    print("Python path:", sys.path)
    sys.exit(1)

if __name__ == '__main__':
    print("Starting dialog_manager_node...")
    dialog_manager_main()