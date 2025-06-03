#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class CameraVisionNode(Node):
    def __init__(self):
        super().__init__('camera_vision')
        self.publisher_ = self.create_publisher(String, 'vision_status', 10)
        self.image_publisher_ = self.create_publisher(Image, 'processed_image', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.bridge = CvBridge()
        self.get_logger().info('Camera Vision Node initialized')

    def timer_callback(self):
        # Create a simple test image (black background with white text)
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.putText(img, "Camera Vision Active", (50, 240), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        
        # Convert the image to a ROS message
        img_msg = self.bridge.cv2_to_imgmsg(img, "bgr8")
        
        # Publish the image
        self.image_publisher_.publish(img_msg)
        
        # Publish status
        status_msg = String()
        status_msg.data = 'Camera vision system active'
        self.publisher_.publish(status_msg)
        
        self.get_logger().debug('Published camera vision status and test image')

def main(args=None):
    rclpy.init(args=args)
    node = CameraVisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
