### Nevil-picar 2.0
SPECIFICATION

---
##GOAL:

Nevil-PiCar-X will be a fully functional ROS 2 robot:

- Conversational companion

- Autonomous driving 

- Obstacle avoidance 

- Camera vision 

- Voice commands 

- Modular ROS2 and PRE architecture

- Multi-Threaded Executor Launcher

- PREEMPT-RT integration for Real Time behaviors
---
##Summary: Ensuring Simultaneity in ROS 2
| **Technique**                     | **Use Case**                                                  |
| --------------------------------- | ------------------------------------------------------------- |
| **MultiThreadedExecutor**         | Run multiple nodes simultaneously                             |
| **Callbacks & Topics**            | Trigger functions only when data is received                  |
| **Actions**                       | Handle long-running tasks like navigation                     |
| **Timers**                        | Run functions periodically (e.g., voice recognition every 5s) |
| **Real-Time Priority Scheduling** | Ensure critical tasks run first                               |
---

##System Overview
Hardware:
Raspberry Pi 4/5 → Runs ROS 2 & AI processing

PiCar-X (SunFounder) → Motor & servo control

Ultrasonic Sensor → Obstacle detection

Camera Module → Computer vision, object tracking

Microphone & Speaker → Voice recognition & text-to-speech

IMU (Inertial Measurement Unit) → Motion stabilization

Software Stack:
ROS 2 (Humble) → Middleware for communication

PREEMPT-RT Kernel → Real-time performance

Cyclone DDS → Low-latency messaging

OpenCV + YOLO → Vision-based object detection

SpeechRecognition + pyttsx3 → Voice commands & responses


---
##Using Multi-Threaded Executor - CODE SAMPLES FOLLOW
To run multiple callback functions at the same time, use multi-threading.

```

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

#Using Callbacks to Trigger Nodes
Each subscriber node reacts to new messages asynchronously.

```
def avoid_callback(self, msg):
    twist = Twist()
    if msg.range < 0.15:
        twist.linear.x = 0.0  # Stop if obstacle detected
        twist.angular.z = 0.5  # Turn right
    else:
        twist.linear.x = 0.2  # Move forward
    self.publisher.publish(twist)
```
 Trigger happens only when sensor data arrives.
---
#Motion Control (Listens to Navigation)
```
def move_callback(self, msg):
    self.px.set_speed(msg.linear.x * 50)
    self.px.set_dir_servo_angle(msg.angular.z * 30)
```
 Motion only runs when /cmd_vel receives a new command.
---
#Using Actions for Long-Running Tasks
For tasks like driving to a waypoint, use ROS 2 Actions (gives feedback, can be canceled).
```
from action_msgs.msg import GoalStatus
from rclpy.action import ActionServer

class DriveToPoint(Node):
    def __init__(self):
        super().__init__('drive_to_point')
        self.action_server = ActionServer(self, Drive, 'drive_action', self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Driving to point...')
        # Move robot until goal is reached
        result = Drive.Result()
        goal_handle.succeed()
        return result
```
The robot executes actions while still reacting to real-time sensor data.
---
# Launching Everything Together
Modify the launch file to use a MultiThreadedExecutor:
```
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    return LaunchDescription([
        Node(package='picar_x', executable='motion_control'),
        Node(package='picar_x', executable='obstacle_avoidance'),
        Node(package='picar_x', executable='navigation'),
        Node(package='picar_x', executable='camera_vision'),
        Node(package='picar_x', executable='voice_control', parameters=[{"rate": 5}]),
    ])
'''

##ROS 2 Nodes & Architecture
Node Breakdown: 
Each function is handled by a separate ROS 2 node for modularity.

ros2_ws/src/nevil/
│── launch/            # Launch multiple nodes together
│   ├── picar_x_launch.py
│── scripts/           # Python scripts for ROS nodes
│   ├── motion_control.py     # Controls motors and servos
│   ├── obstacle_avoidance.py # Uses ultrasonic sensor
│   ├── navigation.py         # Path planning
│   ├── camera_vision.py      # Object tracking & line following
│   ├── voice_control.py      # Speech-to-text & text-to-speech
│── config/            # Config files (YAML for PID tuning, etc.)
│── urdf/              # Robot description (URDF for simulation)
│── msg/               # Custom ROS messages (if needed)
│── srv/               # Custom ROS services (if needed)
│── CMakeLists.txt     # Build configuration
│── package.xml        # ROS 2 package description

---
##PREEMPT-RT improves real-time behavior by:

Making Linux kernel preemptible (removes long blocking paths)

Allowing deterministic thread scheduling with tools like chrt

Enabling ROS 2 callbacks to execute with much lower latency if you:

Install PREEMPT-RT kernel

Tune the scheduler (sched_rt_runtime_us = -1)

Launch nodes with real-time priority

---

##Integrate PREEMPT-RT in Nevil

- Use chrt or taskset in your launch scripts to enforce priority

- Patch rclpy callbacks to check and log thread scheduling (optional)

- Monitor latency/jitter using cyclictest or ROS 2 latency benchmark tools

---

##Why PREEMPT-RT Is Essential for Nevil
Nevil is meant to be:

- Responsive to voice commands

- Reactive to obstacle data in real-time

- Coordinated in simultaneous actions (camera, movement, speech)

- Conversational, without lag or blocking

- ROS 2 by itself does not guarantee:

- Deterministic callback execution

- Low jitter task scheduling

- Priority-based thread scheduling under stress

#With PREEMPT-RT, the Linux kernel:

Preempts long-running kernel tasks, letting critical real-time threads run on time

Allows real-time scheduling (SCHED_FIFO) for ROS 2 callbacks

Reduces latency to sub-millisecond range

Makes MultiThreadedExecutor actually concurrent and predictable

#Without PREEMPT-RT:
Nevil might lag or stutter during simultaneous motor + voice tasks

Speech recognition might block sensor response

Motion could be delayed by unrelated system processes (e.g., logging, networking)

Real-time obstacle response could fail under load

#With PREEMPT-RT:
Every node can be given reliable execution timing

Callback latencies drop from 10–50ms → <1ms jitter

Nevil becomes a true real-time conversational robot

#A Full Implementation Includes:
PREEMPT-RT kernel installed and running

Real-time configuration of node execution (using chrt or launch file wrappers)

Proper system tuning (e.g., disable CPU throttling, reserve CPU cores, isolate I/O)

ROS 2 nodes written with non-blocking callbacks and priority-aware behavior

---

## PREEMPT-RT Kernel Setup

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install linux-image-rt-arm64
sudo reboot
uname -a
```

---

## Set Scheduling Runtime

```bash
echo -1 | sudo tee /proc/sys/kernel/sched_rt_runtime_us
```

---

## Disable Power Saving

```bash
sudo systemctl mask sleep.target suspend.target hibernate.target hybrid-sleep.target
```

---

## CPU Isolation (Optional)

Add to `/boot/cmdline.txt`:
```
isolcpus=3 nohz_full=3 rcu_nocbs=3
```

---

## Launch Nodes with Real-Time Priority

```bash
sudo chrt -f 90 ros2 run picar_x motion_control
```

---

## Launch Wrapper Script

```bash
#!/bin/bash
sudo chrt -f 90 ros2 launch picar_x picar_x_launch.py
```

---

## Python Launch with Priorities

```python
import os
import subprocess

nodes = [
    ("picar_x", "motion_control", 90),
    ("picar_x", "obstacle_avoidance", 85),
    ("picar_x", "navigation", 80),
    ("picar_x", "camera_vision", 70),
    ("picar_x", "voice_control", 60)
]

for package, executable, priority in nodes:
    cmd = ["sudo", "chrt", "-f", str(priority), "ros2", "run", package, executable]
    subprocess.Popen(cmd)
```

---

## Check Node Priority

```bash
ps -eLo pid,rtprio,cmd | grep ros2
```

---

## Latency Testing

```bash
sudo apt install rt-tests
sudo cyclictest -l1000000 -m -Sp90 -i200 -h400 -q
```

---

## Optional Python Priority Boost (Soft Real-Time Hint)

```python
import os
os.nice(-20)
```

---

## Avoid in Callbacks

```python
# BAD: time.sleep() inside callbacks
# GOOD: use ROS 2 timers or events
```

---

Write a Simple ROS 2 Node
🔹 Python Node (my_robot/scripts/move.py)

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MoveRobot(Node):
    def __init__(self):
        super().__init__('move_robot')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.5, self.move_forward)

    def move_forward(self):
        msg = Twist()
        msg.linear.x = 0.2  # Move forward
        self.publisher_.publish(msg)
        self.get_logger().info("Moving forward!")

def main(args=None):
    rclpy.init(args=args)
    node = MoveRobot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

---

 Update setup.py (inside my_robot/)
Edit my_robot/setup.py to add the script:

entry_points={
    'console_scripts': [
        'move_robot = my_robot.move:main',
    ],
},

---

Build & Source the Package

cd ~/ros2_ws
colcon build
source install/setup.bash

---
Run the Node

ros2 run my_robot move_robot
---


 Implementing the ROS 2 Nodes
Motion Control Node (motion_control.py)
Handles motor speeds, turning, and servo adjustments
```
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import picarx  # SunFounder PiCar-X library

class MotionControl(Node):
    def __init__(self):
        super().__init__('motion_control')
        self.px = picarx.PicarX()
        self.subscription = self.create_subscription(
            Twist, 'cmd_vel', self.move_callback, 10)
        self.subscription

    def move_callback(self, msg):
        self.px.set_speed(msg.linear.x * 50)  # Scale speed
        self.px.set_dir_servo_angle(msg.angular.z * 30)  # Scale turn angle

def main():
    rclpy.init()
    node = MotionControl()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
---
Obstacle Avoidance Node (obstacle_avoidance.py)
Reads ultrasonic sensor & stops motion if needed
```
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist

class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.ultrasonic_sub = self.create_subscription(Range, 'ultrasonic_data', self.avoid_callback, 10)

    def avoid_callback(self, msg):
        twist = Twist()
        if msg.range < 0.15:  # Stop if obstacle detected
            twist.linear.x = 0.0
            twist.angular.z = 0.5  # Turn right
        else:
            twist.linear.x = 0.2  # Move forward
        self.publisher.publish(twist)

def main():
    rclpy.init()
    node = ObstacleAvoidance()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
---
Navigation Node (navigation.py)
🔹 Uses line tracking or GPS pathfinding
```
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist

class Navigation(Node):
    def __init__(self):
        super().__init__('navigation')
        self.subscription = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

    def odom_callback(self, msg):
        # Implement A* or line-following navigation logic
        twist = Twist()
        twist.linear.x = 0.2  # Adjust speed based on pathfinding
        self.publisher.publish(twist)

def main():
    rclpy.init()
    node = Navigation()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---
Camera Vision Node (camera_vision.py)
Uses OpenCV for face/object detection
```
import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CameraVision(Node):
    def __init__(self):
        super().__init__('camera_vision')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, 'camera/image_raw', self.process_image, 10)

    def process_image(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        # Run OpenCV processing (e.g., object detection)
        cv2.imshow("PiCar-X Camera", frame)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = CameraVision()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
---
Voice Control Node (voice_control.py)
Uses speech recognition & text-to-speech
```
import rclpy
from rclpy.node import Node
import speech_recognition as sr
import pyttsx3
from std_msgs.msg import String

class VoiceControl(Node):
    def __init__(self):
        super().__init__('voice_control')
        self.publisher = self.create_publisher(String, 'voice_command', 10)
        self.engine = pyttsx3.init()
        self.recognizer = sr.Recognizer()
        self.timer = self.create_timer(5, self.listen)

    def listen(self):
        with sr.Microphone() as source:
            try:
                audio = self.recognizer.listen(source)
                command = self.recognizer.recognize_google(audio).lower()
                self.publisher.publish(String(data=command))
                self.engine.say("Command received: " + command)
                self.engine.runAndWait()
            except:
                pass

def main():
    rclpy.init()
    node = VoiceControl()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
---
Running the System with a Launch File
Launch all nodes together
```
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='picar_x', executable='motion_control'),
        Node(package='picar_x', executable='obstacle_avoidance'),
        Node(package='picar_x', executable='navigation'),
        Node(package='picar_x', executable='camera_vision'),
        Node(package='picar_x', executable='voice_control'),
    ])
```
---
PREEMPT-RT Coding Best Practices:

| Best Practice                           | Why It Matters                                                          |
| --------------------------------------- | ----------------------------------------------------------------------- |
| **Use `MultiThreadedExecutor`**      | Enables ROS 2 to utilize PREEMPT-RT’s preemptible threads               |
| **Avoid `time.sleep()` in callbacks** | Blocks real-time threads; use `create_timer()` or event-driven logic    |
| **Measure latency using timestamps** | Log timestamps before/after pub/sub to measure execution lag            |
| **Minimize callback duration**        | Short callbacks reduce risk of missed deadlines                         |
| **Set thread priority via `chrt`**   | Outside Python, set real-time policy on each node process               |
| **Minimize garbage collection**      | Python’s GC can introduce jitter; monitor or tweak it in critical nodes |
---
