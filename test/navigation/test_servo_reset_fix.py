#!/usr/bin/env python3
"""
Test script to verify that the servo reset fix prevents wheels from staying skewed.

This test simulates the problematic actions that were leaving wheels at 30 degrees
and verifies that they now properly reset to 0 degrees.
"""

import unittest
from unittest.mock import Mock, MagicMock, patch
import sys
import os

# Add the navigation module to the path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../src/nevil_navigation'))

class TestServoResetFix(unittest.TestCase):
    """Test that servo reset functionality works correctly"""
    
    def setUp(self):
        """Set up test fixtures"""
        # Mock the car instance
        self.mock_car = Mock()
        self.mock_car.set_dir_servo_angle = Mock()
        self.mock_car.set_cam_pan_angle = Mock()
        self.mock_car.set_cam_tilt_angle = Mock()
        self.mock_car.reset = Mock()
        self.mock_car.stop = Mock()
        
        # Mock ROS2 node
        with patch('rclpy.node.Node.__init__'):
            from nevil_navigation.picar_actions import PicarActions
            self.picar_actions = PicarActions(car_instance=self.mock_car)
            self.picar_actions.get_logger = Mock(return_value=Mock())
    
    def test_turn_left_in_place_resets_wheels(self):
        """Test that turn_left_in_place properly resets wheels to 0 degrees"""
        # Execute the action
        self.picar_actions.turn_left_in_place()
        
        # Verify the sequence: set to -30, then reset to 0
        calls = self.mock_car.set_dir_servo_angle.call_args_list
        self.assertEqual(len(calls), 2, "Should have two servo angle calls")
        self.assertEqual(calls[0][0][0], -30, "First call should set angle to -30")
        self.assertEqual(calls[1][0][0], 0, "Second call should reset angle to 0")
    
    def test_turn_right_in_place_resets_wheels(self):
        """Test that turn_right_in_place properly resets wheels to 0 degrees"""
        # Execute the action
        self.picar_actions.turn_right_in_place()
        
        # Verify the sequence: set to 30, then reset to 0
        calls = self.mock_car.set_dir_servo_angle.call_args_list
        self.assertEqual(len(calls), 2, "Should have two servo angle calls")
        self.assertEqual(calls[0][0][0], 30, "First call should set angle to 30")
        self.assertEqual(calls[1][0][0], 0, "Second call should reset angle to 0")
    
    def test_keep_think_resets_servos(self):
        """Test that keep_think properly resets all servos"""
        # Execute the action
        self.picar_actions.keep_think()
        
        # Verify that reset was called at the beginning and end
        self.assertEqual(self.mock_car.reset.call_count, 2, "Reset should be called twice")
    
    def test_initialize_servos_sets_all_to_zero(self):
        """Test that initialize_servos sets all servos to 0 degrees"""
        # Execute the initialization
        self.picar_actions.initialize_servos()
        
        # Verify all servos are set to 0
        self.mock_car.set_dir_servo_angle.assert_called_with(0)
        self.mock_car.set_cam_pan_angle.assert_called_with(0)
        self.mock_car.set_cam_tilt_angle.assert_called_with(0)


class TestNavigationNodeServoReset(unittest.TestCase):
    """Test that NavigationNode properly calls servo reset"""
    
    def setUp(self):
        """Set up test fixtures"""
        # Mock dependencies
        self.mock_car = Mock()
        self.mock_picar_actions = Mock()
        self.mock_picar_actions.initialize_servos = Mock()
        
        # Mock ROS2 components
        with patch('rclpy.node.Node.__init__'), \
             patch('rclpy.ok', return_value=True), \
             patch.object(sys, 'path'):
            
            # Mock the imports
            with patch.dict('sys.modules', {
                'nevil_navigation.picarx': Mock(),
                'nevil_interfaces_ai_msgs.msg': Mock(),
                'robot_hat.utils': Mock(),
                'robot_hat': Mock()
            }):
                from nevil_navigation.navigation_node import NavigationNode
                
                # Create a minimal navigation node for testing
                self.nav_node = NavigationNode.__new__(NavigationNode)
                self.nav_node.car = self.mock_car
                self.nav_node.picar_actions = self.mock_picar_actions
                self.nav_node.get_logger = Mock(return_value=Mock())
    
    def test_action_execution_calls_servo_reset(self):
        """Test that action execution calls initialize_servos"""
        # Mock the action method
        mock_action = Mock()
        self.mock_picar_actions.stop = mock_action
        
        # Execute an action
        self.nav_node.execute_action("stop", {})
        
        # Verify that initialize_servos was called after the action
        self.mock_picar_actions.initialize_servos.assert_called_once()
    
    def test_emergency_stop_calls_servo_reset(self):
        """Test that emergency stop calls initialize_servos"""
        # Mock ROS2 context
        with patch('rclpy.ok', return_value=True):
            # Execute emergency stop
            self.nav_node.stop_robot()
        
        # Verify that initialize_servos was called
        self.mock_picar_actions.initialize_servos.assert_called_once()


if __name__ == '__main__':
    print("🧪 Testing Servo Reset Fix")
    print("=" * 50)
    
    # Run the tests
    unittest.main(verbosity=2)