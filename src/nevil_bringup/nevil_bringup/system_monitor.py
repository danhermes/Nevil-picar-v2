#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import psutil
import time

class SystemMonitor(Node):
    def __init__(self):
        super().__init__('system_monitor')
        self.publisher_ = self.create_publisher(String, 'system_status', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('System Monitor initialized')

    def timer_callback(self):
        # Get system stats
        cpu_percent = psutil.cpu_percent()
        memory = psutil.virtual_memory()
        disk = psutil.disk_usage('/')
        
        # Create status message
        status_msg = String()
        status_msg.data = f"CPU: {cpu_percent}% | Memory: {memory.percent}% | Disk: {disk.percent}%"
        
        # Publish status
        self.publisher_.publish(status_msg)
        self.get_logger().debug(f'Published system status: {status_msg.data}')

def main(args=None):
    rclpy.init(args=args)
    monitor = SystemMonitor()
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
