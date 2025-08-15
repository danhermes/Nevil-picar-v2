# Nevil-picar v2.0 Setup Guide

## Quick Python Dependencies Installation

### Option 1: Install all dependencies at once
```bash
source nevil2env/bin/activate
pip install -r requirements.txt
```

### Option 2: Install core dependencies individually
```bash
source nevil2env/bin/activate
pip install opencv-python pygame numpy openai python-dotenv pyaudio
```

## Hardware-specific Dependencies (Raspberry Pi)
```bash
# Install robot-hat for hardware control
pip install robot-hat

# Install picar-x libraries
pip install picar-x
```

## System Dependencies (Ubuntu/Debian)
```bash
# Audio support
sudo apt-get install portaudio19-dev python3-pyaudio

# Development tools (optional)
sudo apt-get install python3-dev build-essential
```

## Build ROS2 Packages
```bash
# Build all packages
./nevil build

# Or build specific packages
./nevil build --packages-select nevil_perception nevil_navigation
```

## Launch System
```bash
# Launch full system
./nevil launch nevil_bringup physical_robot.launch.py

# Launch minimal system  
./nevil launch nevil_bringup minimal_system.launch.py
```
