#!/bin/bash

# Source the ROS2 setup file
source ~/nevil/src/install/setup.bash

# Run the hardware interface test
python3 -c "from nevil_realtime.rt_hardware_interface import main; main()"