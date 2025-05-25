# Nevil: PREEMPT-RT Integration into ROS 2 Architecture

## Overview
This document outlines the full specification for integrating the PREEMPT-RT real-time Linux kernel into Nevil's ROS 2 architecture. It includes installation, system configuration, node prioritization, runtime tools, and optional enhancements to ensure soft real-time operation on a Raspberry Pi running ROS 2 (Humble).

---

## 1. Kernel Installation: PREEMPT-RT on Raspberry Pi

### Step 1: Install PREEMPT-RT Kernel
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

### Step 2: Reboot & Confirm
```bash
uname -a  # Look for "PREEMPT RT" in kernel name
```

---

## 2. System Tuning for Real-Time Performance

### Set Real-Time Scheduling Runtime
```bash
echo -1 | sudo tee /proc/sys/kernel/sched_rt_runtime_us
```

### Disable Power Saving Features (CPU Throttle, C-States)
```bash
sudo systemctl mask sleep.target suspend.target hibernate.target hybrid-sleep.target
```

### Optional: Isolate CPU Core
Add to `/boot/cmdline.txt`:
```
isolcpus=3 nohz_full=3 rcu_nocbs=3
```

---

## 3. Prioritize ROS 2 Node Execution with `chrt`

### Run a Node with Real-Time Priority
```bash
sudo chrt -f 90 ros2 run picar_x motion_control
```

Or wrap in a launch script:
```bash
#!/bin/bash
sudo chrt -f 90 ros2 launch picar_x picar_x_launch.py
```

---

## 4. Recommended Node Design Best Practices

| Practice | Description |
|----------|-------------|
| Short Callbacks | Avoid blocking operations inside subscriber callbacks |
| Avoid time.sleep() | Use ROS 2 timers instead to avoid thread blocking |
| MultiThreadedExecutor | Enables concurrency across nodes |
| Measure latency | Timestamp at publish and receive to measure delay |
| Use real-time priority | Run critical nodes (e.g. motion, avoidance) via `chrt` |

---

## 5. ROS 2 Launch Example with Priority Scheduling

### Python Wrapper (`launch_with_priority.py`)
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

## 6. Diagnostic Tools

### Check Latency
```bash
sudo apt install rt-tests
sudo cyclictest -l1000000 -m -Sp90 -i200 -h400 -q
```

### Monitor Node Priority
```bash
ps -eLo pid,rtprio,cmd | grep ros2
```

---

## 7. Optional Enhancements

- Use C++ for hard real-time control if Python adds too much jitter
- Offload PID control to STM32 or Arduino via CAN/UART
- Implement watchdog timers to restart unresponsive nodes

---

## Summary

PREEMPT-RT transforms Nevil from a hobbyist robot into a soft real-time robotics platform. With proper kernel, tuning, and launch strategy, ROS 2 nodes can meet latency-sensitive demands for navigation, speech, and interaction without unexpected delays or hangs.
