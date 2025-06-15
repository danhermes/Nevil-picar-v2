## 🧠 ROS 2 + `sudo` = ⚠️ Trouble

### 🧵 Issue Summary
Using `sudo` with `ros2 launch` caused environment variables like `PYTHONPATH` to be lost, resulting in:
```
ModuleNotFoundError: No module named 'rclpy'
```

---

### 🪓 Root Cause
Running `ros2 launch` with `sudo` **does not preserve the current user's environment** (even with `-E` unless explicitly managed). ROS 2 relies heavily on environment variables such as:

- `PYTHONPATH`
- `AMENT_PREFIX_PATH`
- `LD_LIBRARY_PATH`
- `COLCON_PREFIX_PATH`

When run with `sudo`, these variables are cleared or replaced by root’s defaults, and ROS nodes cannot locate dependencies like `rclpy`.

---

### ✅ Recommended Fix

**Do not run ROS 2 launch with `sudo`. Instead:**

1. Always source ROS 2 environment as normal user:
    ```bash
    source ~/ros2_humble/install/setup.bash
    ```

2. Launch the robot:
    ```bash
    ros2 launch nevil_bringup physical_robot.launch.py
    ```

---

### 🛡️ If Elevated Privileges Are Absolutely Necessary

Only escalate permission inside **specific nodes**, not the whole launch:

**Use:**
```python
prefix='sudo -E'
```

**Example:**
```python
Node(
    package='your_package',
    executable='your_executable',
    name='privileged_node',
    prefix='sudo -E',
    output='screen'
)
```

---

### 🔐 Additional Best Practices

- Use `usermod -aG` to add your user to groups like `dialout`, `gpio`, `audio`
- Use `udev` rules for device access
- Use `setcap` if needing low-level socket/mic access
- Avoid `sudo bash`, `sudo source`, or root shell habits unless you're installing system packages

---

### 🔁 Conclusion

**Quarantine `sudo`.** Keep ROS 2 in userland. Grant permissions with precision. Let nodes handle privilege escalation if absolutely necessary. Stability and sanity will follow.


Do This: Grant CAP_SYS_NICE to ros2
This allows chrt to work without sudo:

bash
Copy
Edit
sudo setcap cap_sys_nice=eip /home/dan/ros2_humble/install/ros2cli/bin/ros2
Confirm it worked:

bash
Copy
Edit
getcap /home/dan/ros2_humble/install/ros2cli/bin/ros2
Expected output:

bash
Copy
Edit
/home/dan/ros2_humble/install/ros2cli/bin/ros2 = cap_sys_nice+eip
🔄 What to Expect Now
No more sudo needed for chrt

Launch will not die with “Operation not permitted”

You're free to assign real-time priorities from user space

🧼 One Caveat
If you rebuild ros2cli or overwrite that binary, you’ll have to re-run setcap. It’s not sticky across rebuilds.