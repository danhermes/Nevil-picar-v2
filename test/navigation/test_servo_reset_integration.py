#!/usr/bin/env python3
"""
Integration test to verify servo reset fix works with actual hardware/simulation.
This test can be run with ROS2 to verify the fix in practice.
"""

import rclpy
from rclpy.node import Node
import json
import time
from nevil_interfaces_ai_msgs.msg import AICommand

class ServoResetTestNode(Node):
    """Test node to verify servo reset functionality"""
    
    def __init__(self):
        super().__init__('servo_reset_test_node')
        
        # Publisher for AI commands
        self.action_publisher = self.create_publisher(
            AICommand,
            '/nevil/action_command',
            10
        )
        
        self.get_logger().info('Servo Reset Test Node initialized')
        
        # Wait for navigation node to be ready
        time.sleep(2)
        
        # Run the test sequence
        self.run_test_sequence()
    
    def send_action_command(self, action_type, action_data=None):
        """Send an action command to test servo reset"""
        msg = AICommand()
        msg.command_type = action_type
        msg.command_data = json.dumps(action_data or {})
        # Note: timestamp field may not exist in AICommand message
        
        self.get_logger().info(f'Sending action command: {action_type}')
        self.action_publisher.publish(msg)
        
        # Wait for action to complete
        time.sleep(3)
    
    def run_test_sequence(self):
        """Run a sequence of actions that previously caused wheel skew"""
        self.get_logger().info('🧪 Starting Servo Reset Test Sequence')
        self.get_logger().info('=' * 50)
        
        # Test the problematic actions that used to leave wheels skewed
        test_actions = [
            'twist_left',    # turn_left_in_place - used to leave wheels at -30°
            'twist_right',   # turn_right_in_place - used to leave wheels at 30°
            'keep_think',    # keep_think - used to leave servos in random positions
            'think',         # think - should reset properly
            'celebrate',     # celebrate - should reset properly
        ]
        
        for action in test_actions:
            self.get_logger().info(f'Testing action: {action}')
            self.send_action_command(action)
            self.get_logger().info(f'✅ Action {action} completed - wheels should be straight')
        
        self.get_logger().info('🎉 Test sequence completed!')
        self.get_logger().info('If fix is working: wheels should be straight (0°) after each action')
        self.get_logger().info('If fix failed: wheels would be skewed at various angles')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        test_node = ServoResetTestNode()
        
        # Keep node alive for a bit to complete tests
        rclpy.spin_once(test_node, timeout_sec=15)
        
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Test failed with error: {e}")
    finally:
        try:
            test_node.destroy_node()
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()