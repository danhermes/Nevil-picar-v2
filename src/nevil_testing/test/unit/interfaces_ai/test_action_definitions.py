#!/usr/bin/env python3

import unittest
import time
from rclpy.serialization import serialize_message, deserialize_message
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import Header

class TestActionDefinitions(unittest.TestCase):
    """Test the action definitions in the nevil_interfaces_ai package."""

    def setUp(self):
        """Set up the test fixture."""
        # Get the action goal, result, and feedback message classes
        try:
            # Try to import any message from the package to check if it's built
            from nevil_interfaces_ai.msg import AICommand
            
            # Now try to get the action message types
            try:
                # Different ROS2 distributions might use different naming conventions
                # Try several possibilities
                
                # Option 1: Standard ROS2 naming with slash
                self.ProcessDialogGoal = get_message('nevil_interfaces_ai/action/ProcessDialog_Goal')
                self.ProcessDialogResult = get_message('nevil_interfaces_ai/action/ProcessDialog_Result')
                self.ProcessDialogFeedback = get_message('nevil_interfaces_ai/action/ProcessDialog_Feedback')
            except (ImportError, AttributeError, ValueError):
                try:
                    # Option 2: Standard ROS2 naming with dot
                    self.ProcessDialogGoal = get_message('nevil_interfaces_ai.action.ProcessDialog_Goal')
                    self.ProcessDialogResult = get_message('nevil_interfaces_ai.action.ProcessDialog_Result')
                    self.ProcessDialogFeedback = get_message('nevil_interfaces_ai.action.ProcessDialog_Feedback')
                except (ImportError, AttributeError, ValueError):
                    # If all attempts fail, skip the tests with a clear message
                    self.skipTest("Action message types not found. The nevil_interfaces_ai package may need to be built with 'colcon build'.")
        except ImportError:
            # If we can't import any message, the package hasn't been built
            self.skipTest("nevil_interfaces_ai package not found. The package needs to be built with 'colcon build'.")

    def test_process_dialog_goal_creation(self):
        """Test creating a ProcessDialog goal message."""
        # Create a ProcessDialog goal
        goal = self.ProcessDialogGoal()
        goal.initial_utterance = "Hello, how can I help you today?"
        goal.context_id = "dialog_001"
        goal.dialog_mode = "conversation"
        goal.timeout = 60.0

        # Verify the goal fields
        self.assertEqual(goal.initial_utterance, "Hello, how can I help you today?")
        self.assertEqual(goal.context_id, "dialog_001")
        self.assertEqual(goal.dialog_mode, "conversation")
        self.assertEqual(goal.timeout, 60.0)

    def test_process_dialog_result_creation(self):
        """Test creating a ProcessDialog result message."""
        # Create a ProcessDialog result
        result = self.ProcessDialogResult()
        result.success = True
        result.message = "Dialog completed successfully"
        result.final_state = "completed"
        result.dialog_summary = "User asked about the weather and received a response."
        result.actions_taken = ["query_weather", "provide_response"]

        # Verify the result fields
        self.assertEqual(result.success, True)
        self.assertEqual(result.message, "Dialog completed successfully")
        self.assertEqual(result.final_state, "completed")
        self.assertEqual(result.dialog_summary, "User asked about the weather and received a response.")
        self.assertEqual(result.actions_taken, ["query_weather", "provide_response"])

    def test_process_dialog_feedback_creation(self):
        """Test creating a ProcessDialog feedback message."""
        # Create a ProcessDialog feedback
        feedback = self.ProcessDialogFeedback()
        feedback.current_state = "processing"
        feedback.last_utterance = "What's the weather like today?"
        feedback.turn_count = 1
        feedback.elapsed_time = 5.2

        # Verify the feedback fields
        self.assertEqual(feedback.current_state, "processing")
        self.assertEqual(feedback.last_utterance, "What's the weather like today?")
        self.assertEqual(feedback.turn_count, 1)
        self.assertEqual(feedback.elapsed_time, 5.2)

    def test_process_dialog_serialization(self):
        """Test serializing and deserializing ProcessDialog messages."""
        # Create a ProcessDialog goal
        goal = self.ProcessDialogGoal()
        goal.initial_utterance = "Hello, how can I help you today?"
        goal.context_id = "dialog_001"
        goal.dialog_mode = "conversation"
        goal.timeout = 60.0
        
        # Serialize and deserialize the goal
        serialized_goal = serialize_message(goal)
        deserialized_goal = deserialize_message(serialized_goal, self.ProcessDialogGoal)
        
        # Verify the deserialized goal
        self.assertEqual(deserialized_goal.initial_utterance, "Hello, how can I help you today?")
        self.assertEqual(deserialized_goal.context_id, "dialog_001")
        self.assertEqual(deserialized_goal.dialog_mode, "conversation")
        self.assertEqual(deserialized_goal.timeout, 60.0)
        
        # Create a ProcessDialog result
        result = self.ProcessDialogResult()
        result.success = True
        result.message = "Dialog completed successfully"
        result.dialog_summary = "User asked about the weather and received a response."
        result.final_state = "completed"
        
        # Serialize and deserialize the result
        serialized_result = serialize_message(result)
        deserialized_result = deserialize_message(serialized_result, self.ProcessDialogResult)
        
        # Verify the deserialized result
        self.assertEqual(deserialized_result.success, True)
        self.assertEqual(deserialized_result.message, "Dialog completed successfully")
        self.assertEqual(deserialized_result.dialog_summary, "User asked about the weather and received a response.")
        self.assertEqual(deserialized_result.final_state, "completed")
        
        # Create a ProcessDialog feedback
        feedback = self.ProcessDialogFeedback()
        feedback.current_state = "processing"
        feedback.last_utterance = "What's the weather like today?"
        feedback.turn_count = 1
        feedback.elapsed_time = 5.2
        
        # Serialize and deserialize the feedback
        serialized_feedback = serialize_message(feedback)
        deserialized_feedback = deserialize_message(serialized_feedback, self.ProcessDialogFeedback)
        
        # Verify the deserialized feedback
        self.assertEqual(deserialized_feedback.current_state, "processing")
        self.assertEqual(deserialized_feedback.last_utterance, "What's the weather like today?")
        self.assertEqual(deserialized_feedback.turn_count, 1)
        self.assertEqual(deserialized_feedback.elapsed_time, 5.2)

    def test_error_handling(self):
        """Test error handling in ProcessDialog result."""
        # Create a ProcessDialog result with an error
        result = self.ProcessDialogResult()
        result.success = False
        result.message = "Dialog processing failed due to a timeout."
        result.final_state = "error"
        result.dialog_summary = "Dialog timed out after waiting for user response."
        result.actions_taken = []
        
        # Verify the error fields
        self.assertEqual(result.success, False)
        self.assertEqual(result.message, "Dialog processing failed due to a timeout.")
        self.assertEqual(result.final_state, "error")
        self.assertEqual(result.dialog_summary, "Dialog timed out after waiting for user response.")
        self.assertEqual(result.actions_taken, [])

if __name__ == '__main__':
    unittest.main()