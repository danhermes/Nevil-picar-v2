#!/usr/bin/env python3

"""
Test PicarActions Real Movement Through Node Infrastructure

This test follows the v1.0 pattern:
1. Use picarx ONLY for initialization (like v1.0 nevil.py line 83)
2. Use picar_actions for ALL movement (like v1.0 action_helper.py)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time
import sys
import os

class PicarActionsRealMovementNode(Node):
    """Node that uses the correct v1.0 pattern: picarx for init, picar_actions for movement."""
    
    def __init__(self):
        super().__init__('picar_actions_real_movement_node')
        
        # Subscribe to cmd_vel for movement commands
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10
        )
        
        # Subscribe to system mode
        self.system_mode_sub = self.create_subscription(
            String, '/system_mode', self.system_mode_callback, 10
        )
        
        # Initialize hardware using the CORRECT v1.0 pattern
        self.car = None
        self.picar_actions = None
        self.system_mode = 'active'
        self.initialize_hardware_v1_pattern()
        
        self.get_logger().info('PicarActions Real Movement Node initialized')
    
    def initialize_hardware_v1_pattern(self):
        """Initialize hardware using v1.0 pattern: picarx for init, picar_actions for movement."""
        self.get_logger().info('Initializing hardware using v1.0 pattern...')
        
        try:
            # Step 1: Use picarx ONLY for initialization (like v1.0 nevil.py)
            from nevil_navigation.picarx import Picarx
            from robot_hat import reset_mcu
            
            # Reset MCU like v1.0
            reset_mcu()
            time.sleep(0.1)
            
            # Create PicarX instance ONLY for initialization
            self.car = Picarx()
            
            # Set safety attributes like v1.0
            self.car.SafeDistance = 30  # 30cm safe distance
            self.car.DangerDistance = 15  # 15cm danger distance
            self.car.speed = 30  # Set default speed
            
            time.sleep(1)  # Sleep like v1.0
            
            # Step 2: Create PicarActions for ALL movement (like v1.0 action_helper)
            from nevil_navigation.picar_actions import PicarActions
            self.picar_actions = PicarActions()
            
            # CRITICAL: Give PicarActions access to the initialized car object
            self.picar_actions.car = self.car
            self.picar_actions.speed = self.car.speed
            
            # THIS IS THE SUCCESS MESSAGE WE WANT TO SEE:
            self.get_logger().info("PiCar hardware initialized successfully")
            self.get_logger().info("PicarActions ready for REAL movement!")
            
            return True
            
        except Exception as e:
            self.car = None
            self.picar_actions = None
            self.get_logger().warn(f"Failed to initialize PiCar hardware: {e}")
            self.get_logger().info("Running in simulation mode without hardware")
            return False
    
    def cmd_vel_callback(self, msg):
        """Handle cmd_vel messages using PicarActions for REAL movement."""
        if self.car is None or self.picar_actions is None:
            self.get_logger().warning('Cannot move: No hardware available')
            return
        
        if self.system_mode != 'active':
            self.get_logger().warning('Cannot move: System not in active mode')
            return
        
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        self.get_logger().info(f'REAL MOVEMENT via PicarActions: linear.x={linear_x:.2f}, angular.z={angular_z:.2f}')
        
        try:
            # Use PicarActions for ALL movement (like v1.0 action_helper)
            if abs(linear_x) > 0.01:  # Forward/backward movement
                if linear_x > 0:
                    # Use PicarActions.move_forward_this_way (like v1.0)
                    self.get_logger().info('EXECUTING: PicarActions.move_forward_this_way on REAL hardware')
                    distance_cm = int(abs(linear_x) * 20)  # Scale to distance
                    self.picar_actions.move_forward_this_way(distance_cm)
                else:
                    # Use PicarActions.move_backward_this_way (like v1.0)
                    self.get_logger().info('EXECUTING: PicarActions.move_backward_this_way on REAL hardware')
                    distance_cm = int(abs(linear_x) * 20)  # Scale to distance
                    self.picar_actions.move_backward_this_way(distance_cm)
            
            elif abs(angular_z) > 0.01:  # Turning movement
                if angular_z > 0:
                    # Use PicarActions.turn_left (like v1.0)
                    self.get_logger().info('EXECUTING: PicarActions.turn_left on REAL hardware')
                    self.picar_actions.turn_left()
                else:
                    # Use PicarActions.turn_right (like v1.0)
                    self.get_logger().info('EXECUTING: PicarActions.turn_right on REAL hardware')
                    self.picar_actions.turn_right()
            
            else:
                # Use PicarActions.stop (like v1.0)
                self.get_logger().info('EXECUTING: PicarActions.stop on REAL hardware')
                self.picar_actions.stop()
            
            self.get_logger().info('REAL MOVEMENT via PicarActions COMPLETED!')
            
        except Exception as e:
            self.get_logger().error(f'Error executing PicarActions movement: {e}')
    
    def system_mode_callback(self, msg):
        """Handle system mode changes."""
        self.system_mode = msg.data
        self.get_logger().info(f'System mode changed to: {self.system_mode}')
        
        if self.system_mode == 'standby' and self.picar_actions:
            self.picar_actions.stop()
            self.get_logger().info('Hardware stopped via PicarActions due to standby mode')


class PicarActionsTestController(Node):
    """Controller node to send movement commands to test PicarActions."""
    
    def __init__(self):
        super().__init__('picar_actions_test_controller')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.system_mode_pub = self.create_publisher(String, '/system_mode', 10)
        
        self.get_logger().info('PicarActions Test Controller initialized')
    
    def send_picar_actions_test_commands(self):
        """Send movement commands to test PicarActions REAL movement."""
        self.get_logger().info('='*70)
        self.get_logger().info('TESTING REAL HARDWARE MOVEMENT VIA PICARACTIONS')
        self.get_logger().info('='*70)
        
        # Wait for nodes to initialize
        time.sleep(3.0)
        
        # Set system to active mode
        mode_msg = String()
        mode_msg.data = 'active'
        self.system_mode_pub.publish(mode_msg)
        self.get_logger().info('Set system mode to active')
        time.sleep(1.0)
        
        # Test 1: Forward movement via PicarActions
        self.get_logger().info('TEST 1: Forward movement via PicarActions')
        cmd = Twist()
        cmd.linear.x = 0.5  # This will trigger move_forward_this_way
        self.cmd_vel_pub.publish(cmd)
        self.get_logger().info('Sent forward command to trigger PicarActions.move_forward_this_way')
        time.sleep(3.0)
        
        # Test 2: Stop via PicarActions
        self.get_logger().info('TEST 2: Stop via PicarActions')
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
        self.get_logger().info('Sent stop command to trigger PicarActions.stop')
        time.sleep(2.0)
        
        # Test 3: Turn left via PicarActions
        self.get_logger().info('TEST 3: Turn left via PicarActions')
        cmd = Twist()
        cmd.angular.z = 0.5  # This will trigger turn_left
        self.cmd_vel_pub.publish(cmd)
        self.get_logger().info('Sent left turn command to trigger PicarActions.turn_left')
        time.sleep(3.0)
        
        # Test 4: Final stop via PicarActions
        self.get_logger().info('TEST 4: Final stop via PicarActions')
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
        self.get_logger().info('Sent final stop command to trigger PicarActions.stop')
        time.sleep(1.0)
        
        self.get_logger().info('='*70)
        self.get_logger().info('PICARACTIONS REAL HARDWARE MOVEMENT TEST COMPLETED')
        self.get_logger().info('='*70)


def main(args=None):
    """Main function."""
    rclpy.init(args=args)
    
    # Create both nodes
    movement_node = PicarActionsRealMovementNode()
    controller_node = PicarActionsTestController()
    
    # Create executor to run both nodes
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(movement_node)
    executor.add_node(controller_node)
    
    try:
        # Start the test in a separate thread
        import threading
        test_thread = threading.Thread(target=controller_node.send_picar_actions_test_commands)
        test_thread.daemon = True
        test_thread.start()
        
        # Spin both nodes
        executor.spin()
        
    except KeyboardInterrupt:
        movement_node.get_logger().info('Test interrupted')
    finally:
        if movement_node.picar_actions:
            movement_node.picar_actions.stop()
        executor.shutdown()
        movement_node.destroy_node()
        controller_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()