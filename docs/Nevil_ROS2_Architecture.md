# Nevil ROS 2 Architecture with PREEMPT-RT (Multi-threaded)

## Overview
Nevil is a conversational, voice-controlled robot built on the SunFounder PiCar-X platform using a Raspberry Pi running ROS 2 (Humble). To ensure real-time responsiveness for navigation, speech interaction, and obstacle avoidance, the system is configured with the PREEMPT-RT kernel and uses a MultiThreadedExecutor in ROS 2.

---

## System Components

### Hardware:
- Raspberry Pi 4/5
- PiCar-X motors and servos
- Ultrasonic sensor
- Camera module
- Microphone (STT input via OpenAI)
- Speaker (TTS output via OpenAI or pyttsx3)
- Optional IMU

### Software Stack:
- ROS 2 Humble
- PREEMPT-RT patched Linux kernel
- OpenAI API for voice interaction
- Cyclone DDS for fast, reliable messaging
- Python nodes with real-time prioritization

---

## ROS 2 Nodes and Their Triggers

| Node                  | Purpose                            | Trigger                  |
|-----------------------|-------------------------------------|--------------------------|
| motion_control.py     | Controls movement and steering     | Subscribed to /cmd_vel   |
| obstacle_avoidance.py | Stops or reroutes Nevil on obstacles | Subscribed to /ultrasonic_data |
| navigation.py         | High-level movement/path planning  | Subscribed to /odom, Timer |
| camera_vision.py      | Visual processing (e.g., tracking) | Subscribed to /camera/image_raw |
| voice_control.py      | STT + TTS interaction              | Timer (interval listening) |

---

## Multi-Threaded Executor Launcher

```python
from rclpy.executors import MultiThreadedExecutor
import rclpy

from my_robot.motion_control import MotionControl
from my_robot.obstacle_avoidance import ObstacleAvoidance
from my_robot.navigation import Navigation
from my_robot.camera_vision import CameraVision
from my_robot.voice_control import VoiceControl

def main():
    rclpy.init()
    motion = MotionControl()
    obstacle = ObstacleAvoidance()
    navigation = Navigation()
    vision = CameraVision()
    voice = VoiceControl()

    executor = MultiThreadedExecutor()
    executor.add_node(motion)
    executor.add_node(obstacle)
    executor.add_node(navigation)
    executor.add_node(vision)
    executor.add_node(voice)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    motion.destroy_node()
    obstacle.destroy_node()
    navigation.destroy_node()
    vision.destroy_node()
    voice.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## PREEMPT-RT Setup (Linux)

```bash
sudo apt install linux-image-rt-amd64
sudo reboot
uname -a  # Confirm RT kernel
```

Boost ROS 2 node priority:
```bash
sudo chrt -f -p 99 $(pgrep -f motion_control)
```

---

## Summary

- **Architecture:** Multi-threaded, ROS 2 with PREEMPT-RT for soft real-time
- **Execution:** Nodes operate asynchronously using topic-based callbacks and timers
- **Design Focus:** Real-time navigation + conversational interaction
- **Deployment Target:** Raspberry Pi with motor/sensor interfaces
