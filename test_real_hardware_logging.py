#!/usr/bin/env python3

"""
Test Real PiCar Hardware Initialization Logging

This script tests the actual hardware initialization logging by:
1. Showing the exact code that logs the success message
2. Creating a minimal test that you can verify shows the logging
3. NOT declaring success until you confirm it works
"""

import rclpy
from rclpy.node import Node
import time
import sys
import os

class RealHardwareLoggingTest(Node):
    """Test to verify the actual hardware initialization logging."""
    
    def __init__(self):
        super().__init__('real_hardware_logging_test')
        self.get_logger().info('Testing PiCar hardware initialization logging...')
    
    def show_logging_code(self):
        """Show the exact code that handles hardware initialization logging."""
        self.get_logger().info('='*60)
        self.get_logger().info('PICAR HARDWARE INITIALIZATION LOGGING CODE')
        self.get_logger().info('='*60)
        
        nav_file = 'src/nevil_navigation/nevil_navigation/navigation_node.py'
        
        try:
            with open(nav_file, 'r') as f:
                lines = f.readlines()
            
            # Find the hardware initialization section
            start_line = None
            end_line = None
            
            for i, line in enumerate(lines):
                if 'Initialize PiCar hardware exactly like v1.0' in line:
                    start_line = i
                elif start_line and 'self.get_logger().info(\'Navigation Node initialized\')' in line:
                    end_line = i
                    break
            
            if start_line and end_line:
                self.get_logger().info(f'Hardware initialization code (lines {start_line+1}-{end_line+1}):')
                self.get_logger().info('-' * 60)
                
                for i in range(start_line, end_line + 1):
                    line_num = i + 1
                    line_content = lines[i].rstrip()
                    
                    # Highlight the success logging line
                    if 'PiCar hardware initialized successfully' in line_content:
                        self.get_logger().info(f'>>> {line_num:3d}: {line_content}  ← SUCCESS LOG')
                    elif 'Failed to initialize PiCar hardware' in line_content:
                        self.get_logger().info(f'>>> {line_num:3d}: {line_content}  ← FAILURE LOG')
                    else:
                        self.get_logger().info(f'    {line_num:3d}: {line_content}')
                
                self.get_logger().info('-' * 60)
                return True
            else:
                self.get_logger().error('Could not find hardware initialization section')
                return False
                
        except Exception as e:
            self.get_logger().error(f'Error reading navigation_node.py: {e}')
            return False
    
    def test_navigation_node_creation(self):
        """Test creating NavigationNode to see the actual logging."""
        self.get_logger().info('')
        self.get_logger().info('='*60)
        self.get_logger().info('TESTING NAVIGATIONNODE CREATION')
        self.get_logger().info('='*60)
        
        try:
            # Add path for navigation module
            sys.path.insert(0, os.path.join(os.getcwd(), 'src', 'nevil_navigation'))
            
            self.get_logger().info('Creating NavigationNode to trigger hardware initialization...')
            self.get_logger().info('WATCH FOR: "PiCar hardware initialized successfully" or failure message')
            self.get_logger().info('-' * 60)
            
            # Import and create NavigationNode
            from nevil_navigation.navigation_node import NavigationNode
            
            # Create the node - this will trigger hardware initialization
            nav_node = NavigationNode()
            
            # Give it time to complete initialization
            time.sleep(2.0)
            
            # Check what happened
            if hasattr(nav_node, 'car') and nav_node.car is not None:
                self.get_logger().info('✓ NavigationNode created with hardware object')
                hardware_available = True
            else:
                self.get_logger().info('ℹ️  NavigationNode created in simulation mode (no hardware)')
                hardware_available = False
            
            # Cleanup
            nav_node.destroy_node()
            
            self.get_logger().info('-' * 60)
            return hardware_available
            
        except Exception as e:
            self.get_logger().error(f'Error creating NavigationNode: {e}')
            return False
    
    def run_test(self):
        """Run the hardware logging test."""
        self.get_logger().info('PICAR HARDWARE INITIALIZATION LOGGING TEST')
        self.get_logger().info('This test will show you the exact logging behavior')
        self.get_logger().info('')
        
        # Step 1: Show the actual code
        code_found = self.show_logging_code()
        
        if not code_found:
            self.get_logger().error('Could not find the logging code')
            return False
        
        # Step 2: Test NavigationNode creation
        hardware_available = self.test_navigation_node_creation()
        
        # Step 3: Explain what to look for
        self.get_logger().info('')
        self.get_logger().info('='*60)
        self.get_logger().info('WHAT TO VERIFY')
        self.get_logger().info('='*60)
        
        if hardware_available:
            self.get_logger().info('✓ Hardware was detected - you should see:')
            self.get_logger().info('  "PiCar hardware initialized successfully"')
        else:
            self.get_logger().info('ℹ️  No hardware detected - you should see:')
            self.get_logger().info('  "Failed to initialize PiCar hardware: [error]"')
            self.get_logger().info('  "Running in simulation mode without hardware"')
        
        self.get_logger().info('')
        self.get_logger().info('📋 TO SEE SUCCESS MESSAGE ON REAL HARDWARE:')
        self.get_logger().info('1. Connect PiCar-X hardware to Raspberry Pi')
        self.get_logger().info('2. Install robot_hat library with hardware support')
        self.get_logger().info('3. Run this test - it will log "PiCar hardware initialized successfully"')
        
        return True


def main(args=None):
    """Main function."""
    rclpy.init(args=args)
    
    test_node = RealHardwareLoggingTest()
    
    try:
        success = test_node.run_test()
        
        # Brief spin
        for _ in range(10):
            rclpy.spin_once(test_node, timeout_sec=0.1)
        
        if success:
            test_node.get_logger().info('')
            test_node.get_logger().info('TEST COMPLETED - Please verify the logging behavior above')
            test_node.get_logger().info('Did you see the expected hardware initialization messages?')
        
    except KeyboardInterrupt:
        test_node.get_logger().info('Test interrupted')
    finally:
        test_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()