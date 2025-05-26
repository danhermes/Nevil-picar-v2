# Nevil Perception

Sensor processing and environment perception for Nevil-picar v2.0.

## Description

The `nevil_perception` package provides sensor processing and environment perception capabilities for the Nevil-picar robot. It handles:

- Camera image processing and object detection
- Ultrasonic sensor data processing for obstacle detection
- Sensor fusion for environmental awareness
- Feature extraction and scene understanding

## Nodes

### Camera Vision Node

The Camera Vision Node (`camera_vision_node`) is responsible for:

- Processing camera images
- Detecting and tracking objects
- Extracting visual features
- Providing visual data for navigation and AI processing

### Obstacle Detection Node

The Obstacle Detection Node (`obstacle_detection_node`) is responsible for:

- Processing ultrasonic sensor data
- Detecting obstacles in the robot's path
- Providing distance measurements
- Triggering obstacle avoidance behaviors

## Usage

```bash
# Launch the perception system
ros2 launch nevil_perception nevil_perception.launch.py
```

## Dependencies

- ROS2 Humble
- OpenCV
- nevil_interfaces
- nevil_core