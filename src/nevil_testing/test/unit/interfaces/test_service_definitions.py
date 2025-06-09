#!/usr/bin/env python3

import unittest
import time
from rclpy.serialization import serialize_message, deserialize_message
from rosidl_runtime_py.utilities import get_service

class TestServiceDefinitions(unittest.TestCase):
    """Test the service definitions in the nevil_interfaces package."""

    def setUp(self):
        """Set up the test fixture."""
        # Get the service classes
        self.CheckObstacle = get_service('nevil_interfaces/CheckObstacle')

    def test_check_obstacle_request_creation(self):
        """Test creating a CheckObstacle request."""
        # Create a CheckObstacle request
        request = self.CheckObstacle.Request()
        request.direction = 0.0  # Forward
        request.max_distance = 5.0
        request.mode = "single"

        # Verify the request fields
        self.assertEqual(request.direction, 0.0)
        self.assertEqual(request.max_distance, 5.0)
        self.assertEqual(request.mode, "single")

    def test_check_obstacle_response_creation(self):
        """Test creating a CheckObstacle response."""
        # Create a CheckObstacle response
        response = self.CheckObstacle.Response()
        response.obstacle_detected = True
        response.distance = 2.5
        response.obstacle_direction = 0.1
        response.confidence = 0.95

        # Verify the response fields
        self.assertEqual(response.obstacle_detected, True)
        self.assertEqual(response.distance, 2.5)
        self.assertEqual(response.obstacle_direction, 0.1)
        self.assertEqual(response.confidence, 0.95)

    def test_check_obstacle_request_serialization(self):
        """Test serializing and deserializing a CheckObstacle request."""
        # Create a CheckObstacle request
        request = self.CheckObstacle.Request()
        request.direction = 0.5  # Slightly to the left
        request.max_distance = 10.0
        request.mode = "continuous"

        # Serialize and deserialize the request
        serialized_request = serialize_message(request)
        deserialized_request = deserialize_message(serialized_request, self.CheckObstacle.Request)

        # Verify the deserialized request
        self.assertEqual(deserialized_request.direction, 0.5)
        self.assertEqual(deserialized_request.max_distance, 10.0)
        self.assertEqual(deserialized_request.mode, "continuous")

    def test_check_obstacle_response_serialization(self):
        """Test serializing and deserializing a CheckObstacle response."""
        # Create a CheckObstacle response
        response = self.CheckObstacle.Response()
        response.obstacle_detected = False
        response.distance = -1.0
        response.obstacle_direction = 0.0
        response.confidence = 0.0

        # Serialize and deserialize the response
        serialized_response = serialize_message(response)
        deserialized_response = deserialize_message(serialized_response, self.CheckObstacle.Response)

        # Verify the deserialized response
        self.assertEqual(deserialized_response.obstacle_detected, False)
        self.assertEqual(deserialized_response.distance, -1.0)
        self.assertEqual(deserialized_response.obstacle_direction, 0.0)
        self.assertEqual(deserialized_response.confidence, 0.0)

    def test_check_obstacle_edge_cases(self):
        """Test edge cases for CheckObstacle service."""
        # Test with negative max_distance
        request = self.CheckObstacle.Request()
        request.direction = 0.0
        request.max_distance = -1.0  # Invalid value
        request.mode = "single"

        # Test with invalid mode
        request2 = self.CheckObstacle.Request()
        request2.direction = 0.0
        request2.max_distance = 5.0
        request2.mode = "invalid_mode"  # Invalid value

        # Test with extreme direction values
        request3 = self.CheckObstacle.Request()
        request3.direction = 3.14159  # ~180 degrees
        request3.max_distance = 5.0
        request3.mode = "single"

        # Verify serialization/deserialization still works for edge cases
        serialized_request = serialize_message(request)
        deserialized_request = deserialize_message(serialized_request, self.CheckObstacle.Request)
        self.assertEqual(deserialized_request.max_distance, -1.0)

        serialized_request2 = serialize_message(request2)
        deserialized_request2 = deserialize_message(serialized_request2, self.CheckObstacle.Request)
        self.assertEqual(deserialized_request2.mode, "invalid_mode")

        serialized_request3 = serialize_message(request3)
        deserialized_request3 = deserialize_message(serialized_request3, self.CheckObstacle.Request)
        self.assertAlmostEqual(deserialized_request3.direction, 3.14159, places=5)

if __name__ == '__main__':
    unittest.main()