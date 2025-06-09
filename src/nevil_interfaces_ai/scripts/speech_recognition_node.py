#!/usr/bin/env python3

import os
import sys

# Add the source directory to PYTHONPATH
src_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, src_dir)

import rclpy
from nevil_interfaces_ai.speech_recognition_node import main

if __name__ == '__main__':
    main()