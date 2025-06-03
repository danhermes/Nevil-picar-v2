#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np

class ObstacleDetectionNode(Node):
    def __init__(self):
        super().__init__('obstacle_detection')
        self.publisher_ = self.create_publisher(String, 'obstacle_status', 10)
        self.obstacle_publisher_ = self.create_publisher(Point, 'obstacle_position', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('Obstacle Detection Node initialized')

    def timer_callback(self):
        # Publish status
        status_msg = String()
        status_msg.data = 'No obstacles detected'
        self.publisher_.publish(status_msg)
        
        # Publish a dummy obstacle position (0, 0, 0)
        obstacle_msg = Point()
        obstacle_msg.x = 0.0
        obstacle_msg.y = 0.0
        obstacle_msg.z = 0.0
        self.obstacle_publisher_.publish(obstacle_msg)
        
        self.get_logger().debug('Published obstacle detection status')

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
