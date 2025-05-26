#!/usr/bin/env python3

import os
import time
import yaml
import numpy as np
import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseStamped, Twist
from sensor_msgs.msg import Image

class TestUtils:
    """
    Utility functions for Nevil-picar v2.0 tests.
    """
    
    @staticmethod
    def load_config(config_file):
        """Load a YAML configuration file."""
        if not os.path.exists(config_file):
            raise FileNotFoundError(f'Configuration file {config_file} not found')
        
        with open(config_file, 'r') as f:
            return yaml.safe_load(f)
    
    @staticmethod
    def get_package_share_directory(package_name):
        """Get the share directory for a package."""
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory(package_name)
    
    @staticmethod
    def create_pose(x, y, z=0.0, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
        """Create a Pose message."""
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw
        return pose
    
    @staticmethod
    def create_pose_stamped(x, y, z=0.0, qx=0.0, qy=0.0, qz=0.0, qw=1.0, frame_id='map', stamp=None):
        """Create a PoseStamped message."""
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = frame_id
        
        if stamp is not None:
            pose_stamped.header.stamp = stamp
        
        pose_stamped.pose = TestUtils.create_pose(x, y, z, qx, qy, qz, qw)
        return pose_stamped
    
    @staticmethod
    def create_twist(linear_x, angular_z, linear_y=0.0, linear_z=0.0, angular_x=0.0, angular_y=0.0):
        """Create a Twist message."""
        twist = Twist()
        twist.linear.x = linear_x
        twist.linear.y = linear_y
        twist.linear.z = linear_z
        twist.angular.x = angular_x
        twist.angular.y = angular_y
        twist.angular.z = angular_z
        return twist
    
    @staticmethod
    def pose_to_dict(pose):
        """Convert a Pose message to a dictionary."""
        return {
            'position': {
                'x': pose.position.x,
                'y': pose.position.y,
                'z': pose.position.z
            },
            'orientation': {
                'x': pose.orientation.x,
                'y': pose.orientation.y,
                'z': pose.orientation.z,
                'w': pose.orientation.w
            }
        }
    
    @staticmethod
    def dict_to_pose(pose_dict):
        """Convert a dictionary to a Pose message."""
        return TestUtils.create_pose(
            pose_dict['position']['x'],
            pose_dict['position']['y'],
            pose_dict['position']['z'],
            pose_dict['orientation']['x'],
            pose_dict['orientation']['y'],
            pose_dict['orientation']['z'],
            pose_dict['orientation']['w']
        )
    
    @staticmethod
    def calculate_distance(pose1, pose2):
        """Calculate the Euclidean distance between two poses."""
        return np.sqrt(
            (pose1.position.x - pose2.position.x) ** 2 +
            (pose1.position.y - pose2.position.y) ** 2 +
            (pose1.position.z - pose2.position.z) ** 2
        )
    
    @staticmethod
    def calculate_angle_difference(pose1, pose2):
        """Calculate the angle difference between two poses."""
        # Convert quaternions to Euler angles
        from tf_transformations import euler_from_quaternion
        
        q1 = [pose1.orientation.x, pose1.orientation.y, pose1.orientation.z, pose1.orientation.w]
        q2 = [pose2.orientation.x, pose2.orientation.y, pose2.orientation.z, pose2.orientation.w]
        
        _, _, yaw1 = euler_from_quaternion(q1)
        _, _, yaw2 = euler_from_quaternion(q2)
        
        # Calculate the angle difference
        angle_diff = yaw2 - yaw1
        
        # Normalize to [-pi, pi]
        while angle_diff > np.pi:
            angle_diff -= 2 * np.pi
        while angle_diff < -np.pi:
            angle_diff += 2 * np.pi
        
        return angle_diff
    
    @staticmethod
    def image_to_cv2(image_msg):
        """Convert a ROS Image message to a CV2 image."""
        bridge = CvBridge()
        return bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
    
    @staticmethod
    def cv2_to_image(cv2_img, encoding='bgr8'):
        """Convert a CV2 image to a ROS Image message."""
        bridge = CvBridge()
        return bridge.cv2_to_imgmsg(cv2_img, encoding=encoding)
    
    @staticmethod
    def detect_objects_in_image(image_msg, object_type='all'):
        """Detect objects in an image."""
        # Convert the image message to a CV2 image
        cv2_img = TestUtils.image_to_cv2(image_msg)
        
        # Implement object detection based on the object type
        # This is a placeholder implementation
        if object_type == 'all':
            # Detect all objects
            # ...
            return []
        elif object_type == 'obstacle':
            # Detect obstacles
            # ...
            return []
        elif object_type == 'marker':
            # Detect markers
            # ...
            return []
        else:
            raise ValueError(f'Unknown object type: {object_type}')
    
    @staticmethod
    def wait_for_condition(condition_func, timeout_sec=5.0, period_sec=0.1):
        """Wait for a condition to be true."""
        start_time = time.time()
        while not condition_func() and time.time() - start_time < timeout_sec:
            time.sleep(period_sec)
        
        return condition_func()
    
    @staticmethod
    def generate_random_pose(x_min, x_max, y_min, y_max):
        """Generate a random pose within the given bounds."""
        x = np.random.uniform(x_min, x_max)
        y = np.random.uniform(y_min, y_max)
        theta = np.random.uniform(-np.pi, np.pi)
        
        # Convert theta to quaternion
        from tf_transformations import quaternion_from_euler
        q = quaternion_from_euler(0, 0, theta)
        
        return TestUtils.create_pose(x, y, 0.0, q[0], q[1], q[2], q[3])
    
    @staticmethod
    def generate_test_trajectory(start_pose, end_pose, num_points=10):
        """Generate a test trajectory between two poses."""
        trajectory = []
        
        # Linear interpolation between start and end positions
        for i in range(num_points):
            t = i / (num_points - 1)
            x = start_pose.position.x + t * (end_pose.position.x - start_pose.position.x)
            y = start_pose.position.y + t * (end_pose.position.y - start_pose.position.y)
            z = start_pose.position.z + t * (end_pose.position.z - start_pose.position.z)
            
            # Spherical linear interpolation between start and end orientations
            from tf_transformations import quaternion_slerp
            
            q1 = [start_pose.orientation.x, start_pose.orientation.y, 
                  start_pose.orientation.z, start_pose.orientation.w]
            q2 = [end_pose.orientation.x, end_pose.orientation.y, 
                  end_pose.orientation.z, end_pose.orientation.w]
            
            q = quaternion_slerp(q1, q2, t)
            
            pose = TestUtils.create_pose(x, y, z, q[0], q[1], q[2], q[3])
            trajectory.append(pose)
        
        return trajectory
    
    @staticmethod
    def create_test_environment(obstacles=None, markers=None):
        """Create a test environment configuration."""
        env_config = {
            'name': 'test_environment',
            'description': 'Test environment for Nevil-picar v2.0',
            'size': {
                'width': 10.0,
                'height': 10.0
            },
            'obstacles': obstacles or [],
            'markers': markers or []
        }
        
        return env_config
    
    @staticmethod
    def save_test_environment(env_config, file_path):
        """Save a test environment configuration to a file."""
        with open(file_path, 'w') as f:
            yaml.dump(env_config, f)
    
    @staticmethod
    def load_test_environment(file_path):
        """Load a test environment configuration from a file."""
        return TestUtils.load_config(file_path)