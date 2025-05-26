#!/usr/bin/env python3

"""
Battery Monitor for Nevil-picar v2.0

This script monitors the battery level and publishes warnings when the battery is low.
"""

import os
import sys
import time
import yaml
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from ament_index_python.packages import get_package_share_directory

class BatteryMonitor(Node):
    """Battery monitor node for Nevil-picar v2.0."""
    
    def __init__(self):
        """Initialize the battery monitor node."""
        super().__init__('battery_monitor')
        
        # Declare parameters
        self.declare_parameter('config_file', '')
        self.declare_parameter('low_battery_threshold', 7.2)  # V
        self.declare_parameter('critical_battery_threshold', 6.8)  # V
        self.declare_parameter('update_rate', 0.2)  # Hz (check every 5 seconds)
        
        # Get parameters
        config_file = self.get_parameter('config_file').value
        self.low_battery_threshold = self.get_parameter('low_battery_threshold').value
        self.critical_battery_threshold = self.get_parameter('critical_battery_threshold').value
        self.update_rate = self.get_parameter('update_rate').value
        
        # Get package directories
        self.nevil_bringup_dir = get_package_share_directory('nevil_bringup')
        
        # Load configuration
        self.config = self.load_config(config_file)
        
        # Initialize variables
        self.battery_level = 0.0
        self.low_battery_warning_sent = False
        self.critical_battery_warning_sent = False
        
        # Create publishers
        self.battery_pub = self.create_publisher(
            Float32,
            '/nevil/system/battery_level',
            10
        )
        
        self.warning_pub = self.create_publisher(
            String,
            '/nevil/system/warnings',
            10
        )
        
        # Create timer for periodic battery checks
        self.timer = self.create_timer(1.0 / self.update_rate, self.check_battery)
        
        self.get_logger().info('Battery monitor initialized')
    
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
    
    def read_battery_level(self):
        """Read the battery level from hardware."""
        # In a real system, this would read from hardware
        # For now, we just simulate it
        
        # Simulate battery discharge over time
        # Start at 12.6V (full charge) and decrease slowly
        current_time = time.time()
        hours_running = (current_time % 3600) / 3600.0  # Cycle every hour
        
        # Simulate a discharge curve from 12.6V to 6.0V
        battery_level = 12.6 - (6.6 * hours_running)
        
        return battery_level
    
    def check_battery(self):
        """Check the battery level and publish warnings if necessary."""
        # Read battery level
        self.battery_level = self.read_battery_level()
        
        # Publish battery level
        battery_msg = Float32()
        battery_msg.data = self.battery_level
        self.battery_pub.publish(battery_msg)
        
        # Check for low battery
        if self.battery_level < self.critical_battery_threshold and not self.critical_battery_warning_sent:
            warning_msg = String()
            warning_msg.data = f"CRITICAL BATTERY LEVEL: {self.battery_level:.2f}V - SHUTDOWN IMMINENT"
            self.warning_pub.publish(warning_msg)
            self.get_logger().error(warning_msg.data)
            self.critical_battery_warning_sent = True
            
            # In a real system, this would trigger an emergency shutdown
            # For now, we just log it
            self.get_logger().error("Battery critically low, emergency shutdown recommended")
            
        elif self.battery_level < self.low_battery_threshold and not self.low_battery_warning_sent:
            warning_msg = String()
            warning_msg.data = f"LOW BATTERY WARNING: {self.battery_level:.2f}V"
            self.warning_pub.publish(warning_msg)
            self.get_logger().warning(warning_msg.data)
            self.low_battery_warning_sent = True
        
        # Reset warnings if battery level increases (e.g., after charging)
        if self.battery_level > self.low_battery_threshold + 0.5:
            self.low_battery_warning_sent = False
            self.critical_battery_warning_sent = False


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    battery_monitor = BatteryMonitor()
    
    try:
        rclpy.spin(battery_monitor)
    except KeyboardInterrupt:
        pass
    finally:
        battery_monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()