# Nevil-picar v2.0: Installation and Setup Guide

This guide provides detailed instructions for installing and setting up the Nevil-picar v2.0 system, including hardware assembly, software installation, and system configuration.

## Table of Contents

- [Hardware Requirements](#hardware-requirements)
- [Hardware Assembly](#hardware-assembly)
- [Software Requirements](#software-requirements)
- [Operating System Installation](#operating-system-installation)
- [PREEMPT-RT Kernel Installation](#preempt-rt-kernel-installation)
- [ROS2 Installation](#ros2-installation)
- [Nevil-picar v2.0 Installation](#nevil-picar-v20-installation)
- [Configuration](#configuration)
- [Testing the Installation](#testing-the-installation)
- [Troubleshooting](#troubleshooting)

## Hardware Requirements

To build the Nevil-picar v2.0 system, you will need the following hardware components:

- **Raspberry Pi 4/5** (8GB RAM recommended)
- **SunFounder PiCar-X** kit
- **Camera Module** (Raspberry Pi Camera v2 or compatible)
- **Ultrasonic Sensor** (HC-SR04 or compatible)
- **Microphone** (USB microphone or compatible)
- **Speaker** (USB speaker or compatible)
- **Optional IMU** (MPU6050 or compatible)
- **MicroSD Card** (32GB or larger recommended)
- **Power Supply** (5V/3A USB-C for Raspberry Pi)
- **Battery Pack** (for mobile operation)

## Hardware Assembly

### 1. Assemble the PiCar-X Platform

Follow the assembly instructions provided with the SunFounder PiCar-X kit to build the basic platform. The assembly typically includes:

1. Building the chassis
2. Installing the motors and wheels
3. Mounting the servo motors
4. Connecting the motor controller board

Refer to the [SunFounder PiCar-X documentation](docs-sunfounder-com-picar-x-en-latest.pdf) for detailed assembly instructions.

### 2. Install the Raspberry Pi

1. Mount the Raspberry Pi on the PiCar-X platform using the provided mounting hardware
2. Connect the Raspberry Pi to the motor controller board using the provided cables
3. Ensure the connections are secure and properly routed

### 3. Install the Sensors

#### Camera Module

1. Connect the camera module to the Raspberry Pi's camera port using the ribbon cable
2. Mount the camera module on the front of the PiCar-X platform
3. Secure the cable to prevent it from interfering with moving parts

#### Ultrasonic Sensor

1. Mount the ultrasonic sensor on the front of the PiCar-X platform
2. Connect the sensor to the appropriate GPIO pins on the Raspberry Pi:
   - VCC to 5V
   - GND to GND
   - TRIG to GPIO23
   - ECHO to GPIO24 (through a voltage divider)

#### Microphone and Speaker

1. Connect the USB microphone to one of the Raspberry Pi's USB ports
2. Connect the USB speaker to another USB port
3. Mount the microphone and speaker on the PiCar-X platform

#### Optional IMU

1. Mount the IMU on the PiCar-X platform
2. Connect the IMU to the Raspberry Pi's I2C pins:
   - VCC to 3.3V
   - GND to GND
   - SDA to GPIO2 (SDA)
   - SCL to GPIO3 (SCL)

### 4. Connect the Power Supply

1. Connect the battery pack to the motor controller board
2. Connect the Raspberry Pi to the power supply
3. Ensure all connections are secure

## Software Requirements

The Nevil-picar v2.0 system requires the following software components:

- **Operating System**: Raspberry Pi OS (64-bit) or Ubuntu 22.04 for Raspberry Pi
- **PREEMPT-RT Kernel**: Real-time Linux kernel for deterministic performance
- **ROS2 Humble**: Robot Operating System 2 middleware
- **Python 3.8+**: Programming language for ROS2 nodes
- **OpenCV**: Computer vision library
- **TensorFlow Lite**: Machine learning library for local AI models
- **OpenAI API**: For cloud-based AI processing

## Operating System Installation

### 1. Prepare the MicroSD Card

1. Download the Raspberry Pi Imager from [https://www.raspberrypi.org/software/](https://www.raspberrypi.org/software/)
2. Insert the MicroSD card into your computer
3. Launch the Raspberry Pi Imager
4. Select "Raspberry Pi OS (64-bit)" or "Ubuntu 22.04 (64-bit)"
5. Select your MicroSD card
6. Click "Write" to flash the operating system to the card

### 2. Configure the Operating System

1. Insert the MicroSD card into the Raspberry Pi
2. Connect a keyboard, mouse, and monitor to the Raspberry Pi
3. Power on the Raspberry Pi
4. Follow the on-screen instructions to complete the initial setup
5. Update the system:
   ```bash
   sudo apt update
   sudo apt upgrade -y
   ```

### 3. Configure Network Settings

1. Configure Wi-Fi:
   ```bash
   sudo raspi-config
   ```
   Navigate to "Network Options" > "Wi-Fi" and follow the prompts

2. Enable SSH (optional):
   ```bash
   sudo raspi-config
   ```
   Navigate to "Interface Options" > "SSH" and select "Enable"

3. Set a static IP address (optional):
   Edit the `/etc/dhcpcd.conf` file:
   ```bash
   sudo nano /etc/dhcpcd.conf
   ```
   Add the following lines:
   ```
   interface wlan0
   static ip_address=192.168.1.100/24
   static routers=192.168.1.1
   static domain_name_servers=192.168.1.1 8.8.8.8
   ```
   Replace the IP addresses with appropriate values for your network

## PREEMPT-RT Kernel Installation

The PREEMPT-RT kernel provides real-time capabilities essential for deterministic performance in Nevil-picar v2.0.

### 1. Install PREEMPT-RT Kernel

On Raspberry Pi OS (Debian/Ubuntu-based):

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install linux-image-rt-arm64
```

If the kernel package isn't available, compile it manually:

```bash
sudo apt install build-essential bc git fakeroot libncurses5-dev bison flex libssl-dev
git clone --depth 1 https://github.com/raspberrypi/linux.git -b rpi-6.1.y
cd linux
zcat /proc/config.gz > .config
make oldconfig
make menuconfig
# Enable: Preemption Model -> Fully Preemptible Kernel (RT)
make -j4 Image modules dtbs
sudo make modules_install
sudo make install
```

### 2. Reboot and Confirm Installation

```bash
sudo reboot
```

After rebooting, verify the PREEMPT-RT kernel is running:

```bash
uname -a  # Look for "PREEMPT RT" in kernel name
```

### 3. Configure Real-Time Settings

1. Set real-time scheduling runtime:
   ```bash
   echo -1 | sudo tee /proc/sys/kernel/sched_rt_runtime_us
   ```

2. Make the setting persistent by adding it to `/etc/sysctl.conf`:
   ```bash
   sudo nano /etc/sysctl.conf
   ```
   Add the following line:
   ```
   kernel.sched_rt_runtime_us = -1
   ```

3. Disable power-saving features:
   ```bash
   sudo systemctl mask sleep.target suspend.target hibernate.target hybrid-sleep.target
   ```

4. Optional: Isolate a CPU core for real-time tasks:
   Edit `/boot/cmdline.txt`:
   ```bash
   sudo nano /boot/cmdline.txt
   ```
   Add the following to the end of the line:
   ```
   isolcpus=3 nohz_full=3 rcu_nocbs=3
   ```

## ROS2 Installation

### 1. Set Locale

```bash
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

### 2. Add ROS2 Repository

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### 3. Install ROS2 Humble

```bash
sudo apt update
sudo apt install ros-humble-ros-base python3-colcon-common-extensions
```

For development tools:

```bash
sudo apt install ros-dev-tools
```

### 4. Install Cyclone DDS

```bash
sudo apt install ros-humble-rmw-cyclonedds-cpp
```

### 5. Configure ROS2 Environment

Add the following to your `~/.bashrc` file:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
source ~/.bashrc
```

## Nevil-picar v2.0 Installation

### 1. Clone the Repository

```bash
mkdir -p ~/nevil_ws/src
cd ~/nevil_ws/src
git clone https://github.com/username/nevil-picar-v2.git
```

### 2. Install Dependencies

```bash
cd ~/nevil_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### 3. Install Python Dependencies

```bash
pip install opencv-python tensorflow-lite pyttsx3 openai numpy
```

### 4. Build the Workspace

```bash
cd ~/nevil_ws
colcon build
source install/setup.bash
```

### 5. Configure OpenAI API

Create a configuration file for the OpenAI API key:

```bash
mkdir -p ~/.config/nevil
nano ~/.config/nevil/config.yaml
```

Add the following content:

```yaml
openai:
  api_key: "your_openai_api_key_here"
```

## Configuration

### 1. Configure Hardware Interfaces

Edit the hardware configuration file:

```bash
nano ~/nevil_ws/src/nevil-picar-v2/src/nevil_core/config/hardware_config.yaml
```

Adjust the parameters according to your hardware setup:

```yaml
hardware:
  motors:
    left_pin: 27
    right_pin: 17
    pwm_frequency: 1000
  servo:
    pin: 23
    min_angle: -45
    max_angle: 45
  ultrasonic:
    trigger_pin: 23
    echo_pin: 24
  camera:
    resolution:
      width: 640
      height: 480
    framerate: 30
```

### 2. Configure ROS2 Parameters

Edit the ROS2 parameters file:

```bash
nano ~/nevil_ws/src/nevil-picar-v2/src/nevil_core/config/ros_params.yaml
```

Adjust the parameters according to your preferences:

```yaml
/**:
  ros__parameters:
    use_sim_time: false

motion_control_node:
  ros__parameters:
    max_linear_speed: 0.5
    max_angular_speed: 1.0
    control_frequency: 20.0

obstacle_avoidance_node:
  ros__parameters:
    min_distance: 0.2
    stop_distance: 0.1
    avoidance_strategy: "turn_and_go"

navigation_node:
  ros__parameters:
    path_planning_algorithm: "a_star"
    map_resolution: 0.05
    update_frequency: 5.0

camera_vision_node:
  ros__parameters:
    object_detection_model: "yolov5"
    confidence_threshold: 0.5
    detection_frequency: 10.0

voice_control_node:
  ros__parameters:
    listening_timeout: 5.0
    voice_activity_threshold: 0.3
    tts_engine: "openai"
```

### 3. Configure Real-Time Priorities

Edit the real-time configuration file:

```bash
nano ~/nevil_ws/src/nevil-picar-v2/src/nevil_realtime/config/rt_config.yaml
```

Adjust the priorities according to your preferences:

```yaml
rt_priorities:
  motion_control_node: 90
  obstacle_avoidance_node: 85
  navigation_node: 80
  camera_vision_node: 70
  voice_control_node: 60
  ai_processing_node: 50
  system_manager_node: 40
```

## Testing the Installation

### 1. Run the System in Simulation Mode

```bash
cd ~/nevil_ws
source install/setup.bash
ros2 launch nevil_simulation nevil_system_with_simulation.launch.py
```

### 2. Run the System on Physical Hardware

```bash
cd ~/nevil_ws
source install/setup.bash
ros2 launch nevil_core nevil_system.launch.py
```

### 3. Test Basic Functionality

1. Test motion control:
   ```bash
   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
   ```

2. Test obstacle detection:
   ```bash
   ros2 topic echo /ultrasonic_data
   ```

3. Test camera vision:
   ```bash
   ros2 topic echo /camera/image_raw
   ```

4. Test voice control:
   ```bash
   ros2 service call /voice_command std_srvs/srv/Trigger
   ```

## Troubleshooting

### Common Issues and Solutions

#### Issue: ROS2 nodes not starting

**Solution**: Check that all dependencies are installed and the workspace is properly sourced:

```bash
cd ~/nevil_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

#### Issue: Hardware not responding

**Solution**: Check hardware connections and permissions:

```bash
# Check GPIO permissions
sudo usermod -a -G gpio $USER
sudo usermod -a -G i2c $USER
sudo reboot
```

#### Issue: Real-time performance issues

**Solution**: Verify PREEMPT-RT kernel is running and properly configured:

```bash
uname -a  # Should show "PREEMPT RT"
cat /proc/sys/kernel/sched_rt_runtime_us  # Should be -1
```

#### Issue: OpenAI API not working

**Solution**: Check API key configuration:

```bash
cat ~/.config/nevil/config.yaml  # Verify API key is correct
```

#### Issue: Camera not working

**Solution**: Check camera connection and enable the camera interface:

```bash
sudo raspi-config
# Navigate to "Interface Options" > "Camera" and select "Enable"
sudo reboot
```

For more troubleshooting information, see the [Troubleshooting Guide](7_troubleshooting.md).