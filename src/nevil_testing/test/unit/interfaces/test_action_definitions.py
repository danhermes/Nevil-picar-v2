#!/usr/bin/env python3

import unittest
import time
from rclpy.serialization import serialize_message, deserialize_message
from rosidl_runtime_py.utilities import get_message
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

class TestActionDefinitions(unittest.TestCase):
    """Test the action definitions in the nevil_interfaces package."""

    def setUp(self):
        """Set up the test fixture."""
        # Check if the nevil_interfaces package has been built
        try:
            # Try to import any message from the package to check if it's built
            from nevil_interfaces.msg import SystemStatus
            
            # Now try to get the action message types
            try:
                # Different ROS2 distributions might use different naming conventions
                # Try several possibilities
                
                # Option 1: Standard ROS2 naming with slash
                self.NavigateToPointGoal = get_message('nevil_interfaces/action/NavigateToPoint_Goal')
                self.NavigateToPointResult = get_message('nevil_interfaces/action/NavigateToPoint_Result')
                self.NavigateToPointFeedback = get_message('nevil_interfaces/action/NavigateToPoint_Feedback')
                
                self.PerformBehaviorGoal = get_message('nevil_interfaces/action/PerformBehavior_Goal')
                self.PerformBehaviorResult = get_message('nevil_interfaces/action/PerformBehavior_Result')
                self.PerformBehaviorFeedback = get_message('nevil_interfaces/action/PerformBehavior_Feedback')
            except (ImportError, AttributeError, ValueError):
                try:
                    # Option 2: Standard ROS2 naming with dot
                    self.NavigateToPointGoal = get_message('nevil_interfaces.action.NavigateToPoint_Goal')
                    self.NavigateToPointResult = get_message('nevil_interfaces.action.NavigateToPoint_Result')
                    self.NavigateToPointFeedback = get_message('nevil_interfaces.action.NavigateToPoint_Feedback')
                    
                    self.PerformBehaviorGoal = get_message('nevil_interfaces.action.PerformBehavior_Goal')
                    self.PerformBehaviorResult = get_message('nevil_interfaces.action.PerformBehavior_Result')
                    self.PerformBehaviorFeedback = get_message('nevil_interfaces.action.PerformBehavior_Feedback')
                except (ImportError, AttributeError, ValueError):
                    # If all attempts fail, skip the tests with a clear message
                    self.skipTest("Action message types not found. The nevil_interfaces package may need to be built with 'colcon build'.")
        except ImportError:
            # If we can't import any message, the package hasn't been built
            self.skipTest("nevil_interfaces package not found. The package needs to be built with 'colcon build'.")

    def test_navigate_to_point_goal_creation(self):
        """Test creating a NavigateToPoint goal message."""
        # Create a pose
        pose = PoseStamped()
        pose.header.stamp.sec = int(time.time())
        pose.header.frame_id = "map"
        pose.pose.position.x = 1.0
        pose.pose.position.y = 2.0
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0

        # Create a NavigateToPoint goal
        goal = self.NavigateToPointGoal()
        goal.target_pose = pose
        goal.linear_velocity = 0.5
        goal.angular_velocity = 0.2
        goal.avoid_obstacles = True
        goal.goal_tolerance = 0.1

        # Verify the goal fields
        self.assertEqual(goal.target_pose.header.frame_id, "map")
        self.assertEqual(goal.target_pose.pose.position.x, 1.0)
        self.assertEqual(goal.target_pose.pose.position.y, 2.0)
        self.assertEqual(goal.linear_velocity, 0.5)
        self.assertEqual(goal.angular_velocity, 0.2)
        self.assertEqual(goal.avoid_obstacles, True)
        self.assertEqual(goal.goal_tolerance, 0.1)

    def test_navigate_to_point_result_creation(self):
        """Test creating a NavigateToPoint result message."""
        # Create a pose
        pose = PoseStamped()
        pose.header.stamp.sec = int(time.time())
        pose.header.frame_id = "map"
        pose.pose.position.x = 0.9
        pose.pose.position.y = 2.1
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0

        # Create a NavigateToPoint result
        result = self.NavigateToPointResult()
        result.success = True
        result.result_code = "SUCCESS"
        result.message = "Goal reached successfully"
        result.final_pose = pose
        result.distance_traveled = 3.5
        result.time_elapsed = 7.2

        # Verify the result fields
        self.assertEqual(result.success, True)
        self.assertEqual(result.result_code, "SUCCESS")
        self.assertEqual(result.message, "Goal reached successfully")
        self.assertEqual(result.final_pose.header.frame_id, "map")
        self.assertEqual(result.final_pose.pose.position.x, 0.9)
        self.assertEqual(result.final_pose.pose.position.y, 2.1)
        self.assertEqual(result.distance_traveled, 3.5)
        self.assertEqual(result.time_elapsed, 7.2)

    def test_navigate_to_point_feedback_creation(self):
        """Test creating a NavigateToPoint feedback message."""
        # Create a pose
        pose = PoseStamped()
        pose.header.stamp.sec = int(time.time())
        pose.header.frame_id = "map"
        pose.pose.position.x = 0.5
        pose.pose.position.y = 1.0
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0

        # Create a NavigateToPoint feedback
        feedback = self.NavigateToPointFeedback()
        feedback.current_pose = pose
        feedback.distance_remaining = 1.5
        feedback.time_elapsed = 3.0
        feedback.estimated_time_remaining = 4.0
        feedback.current_state = "moving"

        # Verify the feedback fields
        self.assertEqual(feedback.current_pose.header.frame_id, "map")
        self.assertEqual(feedback.current_pose.pose.position.x, 0.5)
        self.assertEqual(feedback.current_pose.pose.position.y, 1.0)
        self.assertEqual(feedback.distance_remaining, 1.5)
        self.assertEqual(feedback.time_elapsed, 3.0)
        self.assertEqual(feedback.estimated_time_remaining, 4.0)
        self.assertEqual(feedback.current_state, "moving")

    def test_navigate_to_point_serialization(self):
        """Test serializing and deserializing NavigateToPoint messages."""
        # Create a NavigateToPoint goal
        goal = self.NavigateToPointGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position.x = 3.0
        goal.target_pose.pose.position.y = 4.0
        goal.linear_velocity = 0.3
        goal.angular_velocity = 0.1
        goal.avoid_obstacles = False
        goal.goal_tolerance = 0.2

        # Serialize and deserialize the goal
        serialized_goal = serialize_message(goal)
        deserialized_goal = deserialize_message(serialized_goal, self.NavigateToPointGoal)

        # Verify the deserialized goal
        self.assertEqual(deserialized_goal.target_pose.header.frame_id, "map")
        self.assertEqual(deserialized_goal.target_pose.pose.position.x, 3.0)
        self.assertEqual(deserialized_goal.target_pose.pose.position.y, 4.0)
        self.assertEqual(deserialized_goal.linear_velocity, 0.3)
        self.assertEqual(deserialized_goal.angular_velocity, 0.1)
        self.assertEqual(deserialized_goal.avoid_obstacles, False)
        self.assertEqual(deserialized_goal.goal_tolerance, 0.2)

    def test_perform_behavior_goal_creation(self):
        """Test creating a PerformBehavior goal message."""
        # Create a PerformBehavior goal
        goal = self.PerformBehaviorGoal()
        goal.behavior_id = "beh_001"
        goal.behavior_name = "Follow Path"
        goal.behavior_category = "navigation"
        goal.param_names = ["speed", "distance"]
        goal.param_values = ["0.5", "10.0"]
        goal.duration = 30.0

        # Verify the goal fields
        self.assertEqual(goal.behavior_id, "beh_001")
        self.assertEqual(goal.behavior_name, "Follow Path")
        self.assertEqual(goal.behavior_category, "navigation")
        self.assertEqual(goal.param_names, ["speed", "distance"])
        self.assertEqual(goal.param_values, ["0.5", "10.0"])
        self.assertEqual(goal.duration, 30.0)

    def test_perform_behavior_result_creation(self):
        """Test creating a PerformBehavior result message."""
        # Create a PerformBehavior result
        result = self.PerformBehaviorResult()
        result.success = True
        result.result_code = "SUCCESS"
        result.message = "Behavior executed successfully"
        result.actual_duration = 25.5

        # Verify the result fields
        self.assertEqual(result.success, True)
        self.assertEqual(result.result_code, "SUCCESS")
        self.assertEqual(result.message, "Behavior executed successfully")
        self.assertEqual(result.actual_duration, 25.5)

    def test_perform_behavior_feedback_creation(self):
        """Test creating a PerformBehavior feedback message."""
        # Create a PerformBehavior feedback
        feedback = self.PerformBehaviorFeedback()
        feedback.progress = 0.5
        feedback.time_elapsed = 15.0
        feedback.current_state = "executing"
        feedback.status_message = "Halfway through execution"

        # Verify the feedback fields
        self.assertEqual(feedback.progress, 0.5)
        self.assertEqual(feedback.time_elapsed, 15.0)
        self.assertEqual(feedback.current_state, "executing")
        self.assertEqual(feedback.status_message, "Halfway through execution")

    def test_perform_behavior_serialization(self):
        """Test serializing and deserializing PerformBehavior messages."""
        # Create a PerformBehavior goal
        goal = self.PerformBehaviorGoal()
        goal.behavior_id = "beh_002"
        goal.behavior_name = "Turn Around"
        goal.behavior_category = "navigation"
        goal.param_names = ["angle"]
        goal.param_values = ["180.0"]
        goal.duration = 10.0

        # Serialize and deserialize the goal
        serialized_goal = serialize_message(goal)
        deserialized_goal = deserialize_message(serialized_goal, self.PerformBehaviorGoal)

        # Verify the deserialized goal
        self.assertEqual(deserialized_goal.behavior_id, "beh_002")
        self.assertEqual(deserialized_goal.behavior_name, "Turn Around")
        self.assertEqual(deserialized_goal.behavior_category, "navigation")
        self.assertEqual(deserialized_goal.param_names, ["angle"])
        self.assertEqual(deserialized_goal.param_values, ["180.0"])
        self.assertEqual(deserialized_goal.duration, 10.0)

if __name__ == '__main__':
    unittest.main()