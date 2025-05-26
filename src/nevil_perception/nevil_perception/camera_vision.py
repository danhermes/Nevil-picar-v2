#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np


class CameraVisionNode(Node):
    """
    Camera Vision Node for Nevil-picar v2.0
    
    This node is responsible for:
    - Processing camera images
    - Detecting and tracking objects
    - Extracting visual features
    - Providing visual data for navigation and AI processing
    """
    
    def __init__(self):
        super().__init__('camera_vision_node')
        
        # Initialize parameters
        self.declare_parameter('enable_detection', True)
        self.declare_parameter('detection_threshold', 0.5)
        self.declare_parameter('camera_fps', 15)
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Create subscribers
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        self.system_mode_subscription = self.create_subscription(
            String,
            '/system_mode',
            self.system_mode_callback,
            10
        )
        
        # Create publishers
        self.detections_publisher = self.create_publisher(
            Detection2DArray,
            '/object_detections',
            10
        )
        
        self.processed_image_publisher = self.create_publisher(
            Image,
            '/camera/processed_image',
            10
        )
        
        # Create timers
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # State variables
        self.current_image = None
        self.current_mode = 'standby'
        
        self.get_logger().info('Camera Vision Node initialized')
        
    def image_callback(self, msg):
        """Process incoming camera images"""
        if not self.get_parameter('enable_detection').get_parameter_value().bool_value:
            return
            
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.current_image = cv_image
            
            # Process the image
            self.process_image(cv_image, msg.header)
            
        except CvBridgeError as e:
            self.get_logger().error(f'CV Bridge error: {e}')
    
    def system_mode_callback(self, msg):
        """Handle system mode changes"""
        self.current_mode = msg.data
        
        # If system is in standby mode, disable detection
        if self.current_mode == 'standby':
            self.set_parameters([rclpy.parameter.Parameter(
                'enable_detection', 
                rclpy.parameter.Parameter.Type.BOOL, 
                False
            )])
            self.get_logger().info('System in standby mode, disabling object detection')
        else:
            self.set_parameters([rclpy.parameter.Parameter(
                'enable_detection', 
                rclpy.parameter.Parameter.Type.BOOL, 
                True
            )])
            self.get_logger().info(f'System in {self.current_mode} mode, enabling object detection')
    
    def timer_callback(self):
        """Periodic processing"""
        if self.current_image is not None:
            self.get_logger().debug('Processing image in timer callback')
    
    def process_image(self, cv_image, header):
        """Process the image and detect objects"""
        if cv_image is None:
            return
            
        # In a real implementation, this would run object detection
        # For now, just create a simple detection message
        detections_msg = Detection2DArray()
        detections_msg.header = header
        
        # Example: Add a dummy detection
        # In a real implementation, this would be based on actual detections
        detection = Detection2D()
        detection.header = header
        detection.bbox.center.x = float(cv_image.shape[1] / 2)
        detection.bbox.center.y = float(cv_image.shape[0] / 2)
        detection.bbox.size_x = 100.0
        detection.bbox.size_y = 100.0
        
        # Add a hypothesis (class prediction)
        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = "example_object"
        hypothesis.hypothesis.score = 0.95
        detection.results.append(hypothesis)
        
        detections_msg.detections.append(detection)
        
        # Publish detections
        self.detections_publisher.publish(detections_msg)
        
        # Draw detection on image (for visualization)
        processed_image = cv_image.copy()
        cv2.rectangle(
            processed_image,
            (int(detection.bbox.center.x - detection.bbox.size_x/2),
             int(detection.bbox.center.y - detection.bbox.size_y/2)),
            (int(detection.bbox.center.x + detection.bbox.size_x/2),
             int(detection.bbox.center.y + detection.bbox.size_y/2)),
            (0, 255, 0),
            2
        )
        
        # Publish processed image
        try:
            processed_msg = self.bridge.cv2_to_imgmsg(processed_image, "bgr8")
            processed_msg.header = header
            self.processed_image_publisher.publish(processed_msg)
        except CvBridgeError as e:
            self.get_logger().error(f'Error converting processed image: {e}')


def main(args=None):
    rclpy.init(args=args)
    
    camera_vision_node = CameraVisionNode()
    
    try:
        rclpy.spin(camera_vision_node)
    except KeyboardInterrupt:
        pass
    finally:
        camera_vision_node.get_logger().info('Shutting down Camera Vision Node')
        camera_vision_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()