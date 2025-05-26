#!/usr/bin/env python3

"""
Nevil-picar v2.0 Command-Line Interface

This script provides a command-line interface for managing the Nevil-picar v2.0 system.
"""

import os
import sys
import argparse
import subprocess
import time
import yaml
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from nevil_interfaces.msg import SystemStatus
from ament_index_python.packages import get_package_share_directory

class NevilCLI:
    """Command-line interface for the Nevil-picar v2.0 system."""
    
    def __init__(self):
        """Initialize the CLI."""
        # Get package directories
        self.nevil_bringup_dir = get_package_share_directory('nevil_bringup')
        
        # Parse arguments
        self.args = self.parse_arguments()
        
        # Execute command
        self.execute_command()
    
    def parse_arguments(self):
        """Parse command-line arguments."""
        parser = argparse.ArgumentParser(description='Nevil-picar v2.0 Command-Line Interface')
        
        # Create subparsers for different commands
        subparsers = parser.add_subparsers(dest='command', help='Command to execute')
        
        # Start command
        start_parser = subparsers.add_parser('start', help='Start the system')
        start_parser.add_argument('--config', type=str, help='Path to configuration file')
        start_parser.add_argument('--mode', type=str, choices=['full', 'minimal', 'simulation', 'physical', 'development'],
                                 default='full', help='System mode to start')
        start_parser.add_argument('--sim', action='store_true', help='Use simulation')
        start_parser.add_argument('--environment', type=str, default='empty',
                                 help='Simulation environment (empty, obstacle_course, maze)')
        
        # Stop command
        subparsers.add_parser('stop', help='Stop the system')
        
        # Status command
        subparsers.add_parser('status', help='Show system status')
        
        # Monitor command
        subparsers.add_parser('monitor', help='Monitor system status')
        
        # Mode command
        mode_parser = subparsers.add_parser('mode', help='Set system mode')
        mode_parser.add_argument('mode', type=str, choices=['standby', 'manual', 'autonomous'],
                                help='System mode to set')
        
        # Diagnostics command
        subparsers.add_parser('diagnostics', help='Run system diagnostics')
        
        # Test command
        test_parser = subparsers.add_parser('test', help='Run tests')
        test_parser.add_argument('--type', type=str, choices=['integration', 'system', 'component'],
                                default='integration', help='Type of test to run')
        test_parser.add_argument('--sim', action='store_true', help='Use simulation for tests')
        
        return parser.parse_args()
    
    def execute_command(self):
        """Execute the specified command."""
        if self.args.command == 'start':
            self.start_system()
        elif self.args.command == 'stop':
            self.stop_system()
        elif self.args.command == 'status':
            self.show_status()
        elif self.args.command == 'monitor':
            self.monitor_system()
        elif self.args.command == 'mode':
            self.set_mode()
        elif self.args.command == 'diagnostics':
            self.run_diagnostics()
        elif self.args.command == 'test':
            self.run_tests()
        else:
            print("No command specified. Use --help for usage information.")
    
    def start_system(self):
        """Start the system."""
        print("Starting Nevil-picar v2.0 system...")
        
        # Determine launch file
        if self.args.mode == 'full':
            launch_file = 'full_system.launch.py'
        elif self.args.mode == 'minimal':
            launch_file = 'minimal_system.launch.py'
        elif self.args.mode == 'simulation':
            launch_file = 'simulation.launch.py'
        elif self.args.mode == 'physical':
            launch_file = 'physical_robot.launch.py'
        elif self.args.mode == 'development':
            launch_file = 'development.launch.py'
        else:
            launch_file = 'full_system.launch.py'
        
        # Build launch arguments
        launch_args = []
        if self.args.config:
            launch_args.extend(['config_file:=' + self.args.config])
        if self.args.sim:
            launch_args.extend(['use_sim:=true'])
        else:
            launch_args.extend(['use_sim:=false'])
        if self.args.environment:
            launch_args.extend(['environment:=' + self.args.environment])
        
        # Build command
        cmd = ['ros2', 'launch', 'nevil_bringup', launch_file] + launch_args
        
        # Execute command
        print(f"Executing: {' '.join(cmd)}")
        try:
            subprocess.run(cmd)
        except KeyboardInterrupt:
            print("\nSystem startup interrupted.")
        except Exception as e:
            print(f"Error starting system: {e}")
    
    def stop_system(self):
        """Stop the system."""
        print("Stopping Nevil-picar v2.0 system...")
        
        # Initialize ROS
        rclpy.init()
        
        # Create node
        node = rclpy.create_node('nevil_cli_stop')
        
        # Create publisher
        shutdown_pub = node.create_publisher(
            Bool,
            '/nevil/system/shutdown',
            10
        )
        
        # Publish shutdown message
        msg = Bool()
        msg.data = True
        shutdown_pub.publish(msg)
        
        # Wait for message to be sent
        time.sleep(0.5)
        
        # Clean up
        node.destroy_node()
        rclpy.shutdown()
        
        print("Shutdown signal sent.")
        
        # Kill any remaining ROS processes
        try:
            subprocess.run(['pkill', '-f', 'nevil_'])
        except:
            pass
    
    def show_status(self):
        """Show system status."""
        print("Checking Nevil-picar v2.0 system status...")
        
        # Initialize ROS
        rclpy.init()
        
        # Create node
        node = rclpy.create_node('nevil_cli_status')
        
        # Create subscriber
        status = None
        
        def status_callback(msg):
            nonlocal status
            status = msg
        
        status_sub = node.create_subscription(
            SystemStatus,
            '/nevil/system/status',
            status_callback,
            10
        )
        
        # Wait for status message
        print("Waiting for status message...")
        start_time = time.time()
        while status is None and time.time() - start_time < 5.0:
            rclpy.spin_once(node, timeout_sec=0.1)
        
        # Display status
        if status is None:
            print("No status message received. Is the system running?")
        else:
            print(f"System status:")
            print(f"  Mode: {status.mode}")
            print(f"  Status: {'OK' if status.ok else 'Error'}")
            print(f"  Error code: {status.error_code}")
            print(f"  Error message: {status.error_message}")
            print(f"  Battery level: {status.battery_level:.2f}V")
            print(f"  CPU temperature: {status.cpu_temperature:.1f}°C")
            print(f"  CPU usage: {status.cpu_usage:.1f}%")
            print(f"  Memory usage: {status.memory_usage:.1f}%")
            print(f"  Disk usage: {status.disk_usage:.1f}%")
        
        # Clean up
        node.destroy_node()
        rclpy.shutdown()
    
    def monitor_system(self):
        """Monitor system status."""
        print("Monitoring Nevil-picar v2.0 system status...")
        print("Press Ctrl+C to stop monitoring.")
        
        # Initialize ROS
        rclpy.init()
        
        # Create node
        node = rclpy.create_node('nevil_cli_monitor')
        
        # Create subscriber
        status = None
        
        def status_callback(msg):
            nonlocal status
            status = msg
        
        status_sub = node.create_subscription(
            SystemStatus,
            '/nevil/system/status',
            status_callback,
            10
        )
        
        # Monitor status
        try:
            while True:
                rclpy.spin_once(node, timeout_sec=0.1)
                
                if status is not None:
                    # Clear screen
                    os.system('cls' if os.name == 'nt' else 'clear')
                    
                    # Display status
                    print(f"System status:")
                    print(f"  Mode: {status.mode}")
                    print(f"  Status: {'OK' if status.ok else 'Error'}")
                    print(f"  Error code: {status.error_code}")
                    print(f"  Error message: {status.error_message}")
                    print(f"  Battery level: {status.battery_level:.2f}V")
                    print(f"  CPU temperature: {status.cpu_temperature:.1f}°C")
                    print(f"  CPU usage: {status.cpu_usage:.1f}%")
                    print(f"  Memory usage: {status.memory_usage:.1f}%")
                    print(f"  Disk usage: {status.disk_usage:.1f}%")
                    print("\nPress Ctrl+C to stop monitoring.")
                    
                    # Reset status
                    status = None
                
                time.sleep(1.0)
        except KeyboardInterrupt:
            print("\nMonitoring stopped.")
        
        # Clean up
        node.destroy_node()
        rclpy.shutdown()
    
    def set_mode(self):
        """Set system mode."""
        print(f"Setting system mode to {self.args.mode}...")
        
        # Initialize ROS
        rclpy.init()
        
        # Create node
        node = rclpy.create_node('nevil_cli_mode')
        
        # Create publisher
        mode_pub = node.create_publisher(
            String,
            '/nevil/system/set_mode',
            10
        )
        
        # Create subscriber to monitor mode change
        current_mode = None
        
        def status_callback(msg):
            nonlocal current_mode
            current_mode = msg.mode
        
        status_sub = node.create_subscription(
            SystemStatus,
            '/nevil/system/status',
            status_callback,
            10
        )
        
        # Wait for initial status
        print("Waiting for system status...")
        start_time = time.time()
        while current_mode is None and time.time() - start_time < 5.0:
            rclpy.spin_once(node, timeout_sec=0.1)
        
        if current_mode is None:
            print("No status message received. Is the system running?")
            node.destroy_node()
            rclpy.shutdown()
            return
        
        # Publish mode change
        msg = String()
        msg.data = self.args.mode
        mode_pub.publish(msg)
        
        # Wait for mode change
        print(f"Waiting for mode change from {current_mode} to {self.args.mode}...")
        start_time = time.time()
        while current_mode != self.args.mode and time.time() - start_time < 5.0:
            rclpy.spin_once(node, timeout_sec=0.1)
        
        if current_mode == self.args.mode:
            print(f"System mode changed to {self.args.mode}.")
        else:
            print(f"Timeout waiting for mode change to {self.args.mode}.")
        
        # Clean up
        node.destroy_node()
        rclpy.shutdown()
    
    def run_diagnostics(self):
        """Run system diagnostics."""
        print("Running system diagnostics...")
        
        # Build command
        cmd = ['ros2', 'run', 'nevil_bringup', 'run_diagnostics.py']
        
        # Execute command
        try:
            subprocess.run(cmd)
        except KeyboardInterrupt:
            print("\nDiagnostics interrupted.")
        except Exception as e:
            print(f"Error running diagnostics: {e}")
    
    def run_tests(self):
        """Run tests."""
        print(f"Running {self.args.type} tests...")
        
        # Build command
        cmd = ['ros2', 'launch', 'nevil_bringup', 'integration_test.launch.py',
               f'test_type:={self.args.type}']
        
        if self.args.sim:
            cmd.append('use_sim:=true')
        else:
            cmd.append('use_sim:=false')
        
        # Execute command
        try:
            subprocess.run(cmd)
        except KeyboardInterrupt:
            print("\nTests interrupted.")
        except Exception as e:
            print(f"Error running tests: {e}")


def main():
    """Main entry point."""
    NevilCLI()


if __name__ == '__main__':
    main()