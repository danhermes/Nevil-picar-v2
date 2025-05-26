#!/usr/bin/env python3

import unittest
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from cv_bridge import CvBridge
import numpy as np
import cv2

from nevil_testing.test_base import NevilTestBase

class TestCameraVisionNode(NevilTestBase):
    """
    Unit tests for the camera_vision_node in the nevil_perception package.
    """
    
    def setUp(self):
        """Set up the test."""
        super().setUp()
        
        # Create a publisher to camera/image_raw
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        
        # Create a publisher to camera/camera_info
        self.camera_info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', 10)
        
        # Create a subscription to object_detections
        self.object_detections = None
        self.object_detections_sub = self.create_subscription(
            String,  # Assuming object detections are published as a JSON string
            '/object_detections',
            self.object_detections_callback,
            10
        )
        
        # Create a subscription to processed image
        self.processed_image = None
        self.processed_image_sub = self.create_subscription(
            Image,
            '/camera/processed_image',
            self.processed_image_callback,
            10
        )
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Wait for the camera vision node to be ready
        self.wait_for_camera_vision()
    
    def object_detections_callback(self, msg):
        """Callback for object detections messages."""
        self.object_detections = msg
    
    def processed_image_callback(self, msg):
        """Callback for processed image messages."""
        self.processed_image = msg
    
    def wait_for_camera_vision(self, timeout_sec=5.0):
        """Wait for the camera vision node to be ready."""
        # Publish a test image
        self.publish_test_image()
        
        # Wait for processed image
        if not self.spin_until(lambda: self.processed_image is not None, timeout_sec):
            self.fail('Timed out waiting for processed image')
    
    def publish_test_image(self):
        """Publish a test image."""
        # Create a test image (black with a white rectangle)
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.rectangle(img, (100, 100), (300, 300), (255, 255, 255), -1)
        
        # Convert to ROS Image message
        img_msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        img_msg.header.stamp = self.node.get_clock().now().to_msg()
        img_msg.header.frame_id = 'camera_frame'
        
        # Publish the image
        self.image_pub.publish(img_msg)
        
        # Create and publish camera info
        camera_info = CameraInfo()
        camera_info.header = img_msg.header
        camera_info.height = 480
        camera_info.width = 640
        camera_info.distortion_model = 'plumb_bob'
        camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        camera_info.k = [500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0]
        camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        camera_info.p = [500.0, 0.0, 320.0, 0.0, 0.0, 500.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        
        self.camera_info_pub.publish(camera_info)
    
    def test_image_processing(self):
        """Test that the camera vision node processes images."""
        # Publish a test image
        self.publish_test_image()
        
        # Wait for the processed image
        if not self.spin_until(lambda: self.processed_image is not None, 1.0):
            self.fail('Timed out waiting for processed image')
        
        # Verify the processed image
        self.assertIsNotNone(self.processed_image)
        self.assertEqual(self.processed_image.height, 480)
        self.assertEqual(self.processed_image.width, 640)
    
    def test_object_detection(self):
        """Test that the camera vision node detects objects."""
        # Publish a test image with an object
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.rectangle(img, (100, 100), (300, 300), (255, 255, 255), -1)
        
        # Convert to ROS Image message
        img_msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        img_msg.header.stamp = self.node.get_clock().now().to_msg()
        img_msg.header.frame_id = 'camera_frame'
        
        # Publish the image
        self.image_pub.publish(img_msg)
        
        # Wait for object detections
        if not self.spin_until(lambda: self.object_detections is not None, 1.0):
            self.fail('Timed out waiting for object detections')
        
        # Verify object detections
        self.assertIsNotNone(self.object_detections)
        
        # Reset for next test
        self.object_detections = None
    
    def test_no_camera_info(self):
        """Test behavior when no camera info is available."""
        # Publish a test image without camera info
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.rectangle(img, (100, 100), (300, 300), (255, 255, 255), -1)
        
        # Convert to ROS Image message
        img_msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        img_msg.header.stamp = self.node.get_clock().now().to_msg()
        img_msg.header.frame_id = 'camera_frame'
        
        # Reset processed image
        self.processed_image = None
        
        # Publish the image (without camera info)
        self.image_pub.publish(img_msg)
        
        # Wait for processed image (should still work without camera info)
        if not self.spin_until(lambda: self.processed_image is not None, 1.0):
            self.fail('Timed out waiting for processed image without camera info')
        
        # Verify the processed image
        self.assertIsNotNone(self.processed_image)
    
    def test_different_image_sizes(self):
        """Test behavior with different image sizes."""
        # Test with a smaller image
        img = np.zeros((240, 320, 3), dtype=np.uint8)
        cv2.rectangle(img, (50, 50), (150, 150), (255, 255, 255), -1)
        
        # Convert to ROS Image message
        img_msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        img_msg.header.stamp = self.node.get_clock().now().to_msg()
        img_msg.header.frame_id = 'camera_frame'
        
        # Reset processed image
        self.processed_image = None
        
        # Publish the image
        self.image_pub.publish(img_msg)
        
        # Wait for processed image
        if not self.spin_until(lambda: self.processed_image is not None, 1.0):
            self.fail('Timed out waiting for processed image with different size')
        
        # Verify the processed image
        self.assertIsNotNone(self.processed_image)
        self.assertEqual(self.processed_image.height, 240)
        self.assertEqual(self.processed_image.width, 320)
    
    def test_image_encoding(self):
        """Test behavior with different image encodings."""
        # Test with grayscale image
        img = np.zeros((480, 640), dtype=np.uint8)
        cv2.rectangle(img, (100, 100), (300, 300), 255, -1)
        
        # Convert to ROS Image message
        img_msg = self.bridge.cv2_to_imgmsg(img, encoding='mono8')
        img_msg.header.stamp = self.node.get_clock().now().to_msg()
        img_msg.header.frame_id = 'camera_frame'
        
        # Reset processed image
        self.processed_image = None
        
        # Publish the image
        self.image_pub.publish(img_msg)
        
        # Wait for processed image
        if not self.spin_until(lambda: self.processed_image is not None, 1.0):
            self.fail('Timed out waiting for processed image with different encoding')
        
        # Verify the processed image
        self.assertIsNotNone(self.processed_image)
    
    def test_parameter_changes(self):
        """Test that parameter changes affect processing."""
        # Get the current parameters
        from rclpy.parameter import Parameter
        
        # Create a client to set parameters
        client = self.node.create_client(
            SetParameters,
            '/camera_vision_node/set_parameters'
        )
        
        # Wait for the service to be available
        if not client.wait_for_service(timeout_sec=1.0):
            self.fail('Service not available')
        
        # Set a parameter to change processing behavior
        from rcl_interfaces.msg import Parameter as ParameterMsg
        from rcl_interfaces.msg import ParameterValue
        from rcl_interfaces.msg import ParameterType
        from rcl_interfaces.srv import SetParameters
        
        request = SetParameters.Request()
        
        param = ParameterMsg()
        param.name = 'detection_threshold'
        param.value.type = ParameterType.PARAMETER_DOUBLE
        param.value.double_value = 0.8  # Higher threshold
        
        request.parameters = [param]
        
        # Send the request
        future = client.call_async(request)
        
        # Wait for the response
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=1.0)
        
        # Publish a test image
        self.publish_test_image()
        
        # Wait for object detections
        time.sleep(0.5)  # Give time for processing with new parameters
        
        # Reset the parameter
        param.value.double_value = 0.5  # Default threshold
        request.parameters = [param]
        
        # Send the request
        future = client.call_async(request)
        
        # Wait for the response
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=1.0)

if __name__ == '__main__':
    unittest.main()