#!/usr/bin/env python3

import unittest
import time
from rclpy.serialization import serialize_message, deserialize_message
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import Header

class TestMessageDefinitions(unittest.TestCase):
    """Test the message definitions in the nevil_interfaces package."""

    def setUp(self):
        """Set up the test fixture."""
        # Get the message classes
        self.SystemStatus = get_message('nevil_interfaces/SystemStatus')
        self.BehaviorStatus = get_message('nevil_interfaces/BehaviorStatus')

    def test_system_status_creation(self):
        """Test creating a SystemStatus message."""
        # Create a header
        header = Header()
        header.stamp.sec = int(time.time())
        header.frame_id = "base_link"

        # Create a SystemStatus message
        msg = self.SystemStatus()
        msg.header = header
        msg.mode = "standby"
        msg.battery_level = 0.75
        msg.low_battery = False
        msg.system_ok = True
        msg.active_nodes = ["node1", "node2"]
        msg.error_nodes = []
        msg.has_errors = False
        msg.error_codes = []
        msg.error_messages = []
        msg.cpu_usage = 0.2
        msg.memory_usage = 0.3
        msg.disk_usage = 0.4

        # Verify the message fields
        self.assertEqual(msg.header.frame_id, "base_link")
        self.assertEqual(msg.mode, "standby")
        self.assertEqual(msg.battery_level, 0.75)
        self.assertEqual(msg.low_battery, False)
        self.assertEqual(msg.system_ok, True)
        self.assertEqual(msg.active_nodes, ["node1", "node2"])
        self.assertEqual(msg.error_nodes, [])
        self.assertEqual(msg.has_errors, False)
        self.assertEqual(msg.error_codes, [])
        self.assertEqual(msg.error_messages, [])
        self.assertEqual(msg.cpu_usage, 0.2)
        self.assertEqual(msg.memory_usage, 0.3)
        self.assertEqual(msg.disk_usage, 0.4)

    def test_system_status_serialization(self):
        """Test serializing and deserializing a SystemStatus message."""
        # Create a SystemStatus message
        msg = self.SystemStatus()
        msg.header.stamp.sec = int(time.time())
        msg.header.frame_id = "base_link"
        msg.mode = "autonomous"
        msg.battery_level = 0.5
        msg.low_battery = True
        msg.system_ok = False
        msg.active_nodes = ["node1"]
        msg.error_nodes = ["node2"]
        msg.has_errors = True
        msg.error_codes = ["E001"]
        msg.error_messages = ["Error message"]
        msg.cpu_usage = 0.8
        msg.memory_usage = 0.7
        msg.disk_usage = 0.6

        # Serialize and deserialize the message
        serialized_msg = serialize_message(msg)
        deserialized_msg = deserialize_message(serialized_msg, self.SystemStatus)

        # Verify the deserialized message
        self.assertEqual(deserialized_msg.header.frame_id, "base_link")
        self.assertEqual(deserialized_msg.mode, "autonomous")
        self.assertEqual(deserialized_msg.battery_level, 0.5)
        self.assertEqual(deserialized_msg.low_battery, True)
        self.assertEqual(deserialized_msg.system_ok, False)
        self.assertEqual(deserialized_msg.active_nodes, ["node1"])
        self.assertEqual(deserialized_msg.error_nodes, ["node2"])
        self.assertEqual(deserialized_msg.has_errors, True)
        self.assertEqual(deserialized_msg.error_codes, ["E001"])
        self.assertEqual(deserialized_msg.error_messages, ["Error message"])
        self.assertAlmostEqual(deserialized_msg.cpu_usage, 0.8, places=5)
        self.assertAlmostEqual(deserialized_msg.memory_usage, 0.7, places=5)
        self.assertAlmostEqual(deserialized_msg.disk_usage, 0.6, places=5)

    def test_behavior_status_creation(self):
        """Test creating a BehaviorStatus message."""
        # Create a header
        header = Header()
        header.stamp.sec = int(time.time())
        header.frame_id = "base_link"

        # Create a BehaviorStatus message
        msg = self.BehaviorStatus()
        msg.header = header
        msg.behavior_id = "beh_001"
        msg.behavior_name = "Follow Path"
        msg.behavior_category = "navigation"
        msg.status = "running"
        msg.progress = 0.5
        msg.duration = 10.0
        msg.param_names = ["speed", "distance"]
        msg.param_values = ["0.5", "10.0"]
        msg.success = False
        msg.result_code = ""
        msg.result_message = ""

        # Verify the message fields
        self.assertEqual(msg.header.frame_id, "base_link")
        self.assertEqual(msg.behavior_id, "beh_001")
        self.assertEqual(msg.behavior_name, "Follow Path")
        self.assertEqual(msg.behavior_category, "navigation")
        self.assertEqual(msg.status, "running")
        self.assertEqual(msg.progress, 0.5)
        self.assertEqual(msg.duration, 10.0)
        self.assertEqual(msg.param_names, ["speed", "distance"])
        self.assertEqual(msg.param_values, ["0.5", "10.0"])
        self.assertEqual(msg.success, False)
        self.assertEqual(msg.result_code, "")
        self.assertEqual(msg.result_message, "")

    def test_behavior_status_serialization(self):
        """Test serializing and deserializing a BehaviorStatus message."""
        # Create a BehaviorStatus message
        msg = self.BehaviorStatus()
        msg.header.stamp.sec = int(time.time())
        msg.header.frame_id = "base_link"
        msg.behavior_id = "beh_002"
        msg.behavior_name = "Turn Around"
        msg.behavior_category = "navigation"
        msg.status = "completed"
        msg.progress = 1.0
        msg.duration = 5.0
        msg.param_names = ["angle"]
        msg.param_values = ["180.0"]
        msg.success = True
        msg.result_code = "SUCCESS"
        msg.result_message = "Behavior completed successfully"

        # Serialize and deserialize the message
        serialized_msg = serialize_message(msg)
        deserialized_msg = deserialize_message(serialized_msg, self.BehaviorStatus)

        # Verify the deserialized message
        self.assertEqual(deserialized_msg.header.frame_id, "base_link")
        self.assertEqual(deserialized_msg.behavior_id, "beh_002")
        self.assertEqual(deserialized_msg.behavior_name, "Turn Around")
        self.assertEqual(deserialized_msg.behavior_category, "navigation")
        self.assertEqual(deserialized_msg.status, "completed")
        self.assertEqual(deserialized_msg.progress, 1.0)
        self.assertEqual(deserialized_msg.duration, 5.0)
        self.assertEqual(deserialized_msg.param_names, ["angle"])
        self.assertEqual(deserialized_msg.param_values, ["180.0"])
        self.assertEqual(deserialized_msg.success, True)
        self.assertEqual(deserialized_msg.result_code, "SUCCESS")
        self.assertEqual(deserialized_msg.result_message, "Behavior completed successfully")

if __name__ == '__main__':
    unittest.main()