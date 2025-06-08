#!/usr/bin/env python3

"""
System Monitor for Nevil-picar v2.0

This script monitors the system status and publishes diagnostic information.
"""

import os
import sys
import time
import yaml
import psutil
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from nevil_interfaces.msg import SystemStatus
from ament_index_python.packages import get_package_share_directory

class SystemMonitor(Node):
    """System monitor node for Nevil-picar v2.0."""
    
    def __init__(self):
        """Initialize the system monitor node."""
        super().__init__('system_monitor')
        
        # Declare parameters
        self.declare_parameter('config_file', '')
        self.declare_parameter('physical_mode', False)
        self.declare_parameter('simulation_mode', False)
        self.declare_parameter('development_mode', False)
        self.declare_parameter('minimal_mode', False)
        self.declare_parameter('update_rate', 1.0)  # Hz
        
        # Get parameters
        config_file = self.get_parameter('config_file').value
        self.physical_mode = self.get_parameter('physical_mode').value
        self.simulation_mode = self.get_parameter('simulation_mode').value
        self.development_mode = self.get_parameter('development_mode').value
        self.minimal_mode = self.get_parameter('minimal_mode').value
        self.update_rate = self.get_parameter('update_rate').value
        
        # Get package directories
        self.nevil_bringup_dir = get_package_share_directory('nevil_bringup')
        
        # Load configuration
        self.config = self.load_config(config_file)
        
        # Initialize variables
        self.system_mode = 'standby'
        self.components = {}
        self.battery_level = 0.0
        self.cpu_temperature = 0.0
        
        # Create publishers
        self.status_pub = self.create_publisher(
            SystemStatus,
            '/nevil/system/status',
            10
        )
        
        # Create subscribers
        self.mode_sub = self.create_subscription(
            String,
            '/nevil/system/set_mode',
            self.mode_callback,
            10
        )
        
        self.shutdown_sub = self.create_subscription(
            Bool,
            '/nevil/system/shutdown',
            self.shutdown_callback,
            10
        )
        
        # Create timer for periodic status updates
        self.timer = self.create_timer(1.0 / self.update_rate, self.update_status)
        
        self.get_logger().info('System monitor initialized')
    
    def load_config(self, config_file):
        """Load configuration from file."""
        if not config_file:
            config_file = os.path.join(self.nevil_bringup_dir, 'config', 'default_config.yaml')
        
        try:
            with open(config_file, 'r') as f:
                return yaml.safe_load(f)
        except Exception as e:
            self.get_logger().error(f'Failed to load configuration: {e}')
            return {}
    
    def mode_callback(self, msg):
        """Callback for system mode messages."""
        new_mode = msg.data
        if new_mode in ['standby', 'manual', 'autonomous']:
            self.system_mode = new_mode
            self.get_logger().info(f'System mode changed to {new_mode}')
        else:
            self.get_logger().warning(f'Invalid system mode: {new_mode}')
    
    def shutdown_callback(self, msg):
        """Callback for system shutdown messages."""
        if msg.data:
            self.get_logger().info('System shutdown requested')
            # Publish final status
            self.update_status()
            # Shutdown ROS
            rclpy.shutdown()
    
    def update_status(self):
        """Update and publish system status."""
        # Create status message
        status = SystemStatus()
        status.mode = self.system_mode
        status.ok = True
        status.error_code = 0
        status.error_message = ''
        
        # Get system information
        status.cpu_usage = psutil.cpu_percent()
        status.memory_usage = psutil.virtual_memory().percent
        status.disk_usage = psutil.disk_usage('/').percent
        
        # Get CPU temperature (platform-specific)
        try:
            if self.physical_mode:
                # On Raspberry Pi
                with open('/sys/class/thermal/thermal_zone0/temp', 'r') as f:
                    status.cpu_temperature = float(f.read().strip()) / 1000.0
            else:
                status.cpu_temperature = 0.0
        except:
            status.cpu_temperature = 0.0
        
        # Get battery level (platform-specific)
        if self.physical_mode:
            # Simulate battery level for now
            # In a real system, this would read from hardware
            status.battery_level = 12.0
        else:
            status.battery_level = 0.0
        
        # Check for errors
        if status.cpu_usage > 90.0:
            status.ok = False
            status.error_code = 1
            status.error_message = 'CPU usage too high'
        
        if status.memory_usage > 90.0:
            status.ok = False
            status.error_code = 2
            status.error_message = 'Memory usage too high'
        
        if status.disk_usage > 90.0:
            status.ok = False
            status.error_code = 3
            status.error_message = 'Disk usage too high'
        
        if self.physical_mode and status.cpu_temperature > 80.0:
            status.ok = False
            status.error_code = 4
            status.error_message = 'CPU temperature too high'
        
        if self.physical_mode and status.battery_level < 7.0:
            status.ok = False
            status.error_code = 5
            status.error_message = 'Battery level low'
        
        # Publish status
        self.status_pub.publish(status)


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    system_monitor = SystemMonitor()
    
    try:
        rclpy.spin(system_monitor)
    except KeyboardInterrupt:
        pass
    finally:
        system_monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()