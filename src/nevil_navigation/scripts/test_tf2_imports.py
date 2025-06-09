#!/usr/bin/env python3

import sys

def main():
    print("Testing TF2 imports...")
    
    try:
        import tf2_ros
        print("✓ Successfully imported tf2_ros")
    except ImportError as e:
        print(f"✗ Failed to import tf2_ros: {e}")
        
    try:
        import tf2_geometry_msgs
        print("✓ Successfully imported tf2_geometry_msgs")
    except ImportError as e:
        print(f"✗ Failed to import tf2_geometry_msgs: {e}")
        
    try:
        import tf2_msgs
        print("✓ Successfully imported tf2_msgs")
    except ImportError as e:
        print(f"✗ Failed to import tf2_msgs: {e}")
        
    print("TF2 import test complete")
    
if __name__ == '__main__':
    main()