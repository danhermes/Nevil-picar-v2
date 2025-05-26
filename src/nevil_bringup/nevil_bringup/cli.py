#!/usr/bin/env python3

"""
Nevil-picar v2.0 Command-Line Interface

This module provides a command-line interface for managing the Nevil-picar v2.0 system.
"""

import os
import sys
import argparse
import yaml
import subprocess
import signal
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from nevil_interfaces.msg import SystemStatus
from ament_index_python.packages import get_package_share_directory

class NevilCLI(Node):
    """Command-line interface for the Nevil-picar v2.0 system."""
    
    def __init__(self):
        """Initialize the CLI node."""
        super().__init__('nevil_cli')
        
        # Initialize variables
        self.system_status = None
        self.system_mode = None
        self.processes = []
        
        # Create subscribers
        self.status_sub = self.create_subscription(
            SystemStatus,
            '/nevil/system/status',
            self.status_callback,
            10
        )
        
        # Create publishers
        self.mode_pub = self.create_publisher(
            String,
            '/nevil/system/set_mode',
            10
        )
        
        self.shutdown_pub = self.create_publisher(
            Bool,
            '/nevil/system/shutdown',
            10
        )
        
        # Get package directories
        self.nevil_bringup_dir = get_package_share_directory('nevil_bringup')
        
        # Load configuration
        self.config = self.load_config()
    
    def status_callback(self, msg):
        """Callback for system status messages."""
        self.system_status = msg
        self.system_mode = msg.mode
    
    def load_config(self, config_file=None):
        """Load configuration from file."""
        if config_file is None:
            config_file = os.path.join(self.nevil_bringup_dir, 'config', 'default_config.yaml')
        
        try:
            with open(config_file, 'r') as f:
                return yaml.safe_load(f)
        except Exception as e:
            self.get_logger().error(f'Failed to load configuration: {e}')
            return {}
    
    def launch_system(self, launch_file, args=None):
        """Launch the system using the specified launch file."""
        if args is None:
            args = []
        
        cmd = ['ros2', 'launch', 'nevil_bringup', launch_file] + args
        self.get_logger().info(f'Launching system: {" ".join(cmd)}')
        
        try:
            process = subprocess.Popen(cmd)
            self.processes.append(process)
            return process
        except Exception as e:
            self.get_logger().error(f'Failed to launch system: {e}')
            return None
    
    def shutdown_system(self):
        """Shutdown the system."""
        # Publish shutdown message
        msg = Bool()
        msg.data = True
        self.shutdown_pub.publish(msg)
        
        # Wait for system to shutdown
        time.sleep(2.0)
        
        # Terminate any running processes
        for process in self.processes:
            try:
                process.terminate()
                process.wait(timeout=5.0)
            except Exception as e:
                self.get_logger().warning(f'Failed to terminate process: {e}')
                try:
                    process.kill()
                except:
                    pass
        
        self.processes = []
    
    def set_mode(self, mode):
        """Set the system mode."""
        msg = String()
        msg.data = mode
        self.mode_pub.publish(msg)
        
        # Wait for mode change
        start_time = time.time()
        while time.time() - start_time < 5.0:
            if self.system_mode == mode:
                self.get_logger().info(f'System mode changed to {mode}')
                return True
            time.sleep(0.1)
            rclpy.spin_once(self, timeout_sec=0.1)
        
        self.get_logger().warning(f'Timeout waiting for mode change to {mode}')
        return False
    
    def get_status(self):
        """Get the current system status."""
        # Wait for status message
        start_time = time.time()
        while self.system_status is None and time.time() - start_time < 5.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        return self.system_status
    
    def print_status(self):
        """Print the current system status."""
        status = self.get_status()
        if status is None:
            print("System status: Unknown (no status message received)")
            return
        
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
        
        if hasattr(status, 'components'):
            print("  Components:")
            for component in status.components:
                print(f"    {component.name}: {'OK' if component.ok else 'Error'}")
                if component.error_message:
                    print(f"      Error: {component.error_message}")
    
    def run_diagnostics(self):
        """Run system diagnostics."""
        print("Running system diagnostics...")
        
        # Check if system is running
        status = self.get_status()
        if status is None:
            print("System is not running. Start the system first.")
            return
        
        # Run diagnostics
        cmd = ['ros2', 'run', 'nevil_bringup', 'run_diagnostics.py']
        try:
            subprocess.run(cmd, check=True)
        except Exception as e:
            print(f"Failed to run diagnostics: {e}")
    
    def monitor_system(self):
        """Monitor the system status."""
        print("Monitoring system status (press Ctrl+C to stop)...")
        
        try:
            while True:
                self.print_status()
                print("\nPress Ctrl+C to stop monitoring.")
                time.sleep(1.0)
                for _ in range(10):  # Update every second, but check for messages more frequently
                    rclpy.spin_once(self, timeout_sec=0.1)
        except KeyboardInterrupt:
            print("\nStopped monitoring.")


def main():
    """Main entry point for the CLI."""
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
    
    # Parse arguments
    args = parser.parse_args()
    
    # Initialize ROS
    rclpy.init()
    
    # Create CLI node
    cli = NevilCLI()
    
    try:
        # Execute command
        if args.command == 'start':
            # Determine launch file
            if args.mode == 'full':
                launch_file = 'full_system.launch.py'
            elif args.mode == 'minimal':
                launch_file = 'minimal_system.launch.py'
            elif args.mode == 'simulation':
                launch_file = 'simulation.launch.py'
            elif args.mode == 'physical':
                launch_file = 'physical_robot.launch.py'
            elif args.mode == 'development':
                launch_file = 'development.launch.py'
            else:
                launch_file = 'full_system.launch.py'
            
            # Build launch arguments
            launch_args = []
            if args.config:
                launch_args.extend(['config_file:=' + args.config])
            if args.sim:
                launch_args.extend(['use_sim:=true'])
            else:
                launch_args.extend(['use_sim:=false'])
            if args.environment:
                launch_args.extend(['environment:=' + args.environment])
            
            # Launch system
            cli.launch_system(launch_file, launch_args)
            
            # Wait for system to start
            time.sleep(2.0)
            
            # Print status
            cli.print_status()
            
        elif args.command == 'stop':
            cli.shutdown_system()
            print("System stopped.")
            
        elif args.command == 'status':
            cli.print_status()
            
        elif args.command == 'monitor':
            cli.monitor_system()
            
        elif args.command == 'mode':
            if cli.set_mode(args.mode):
                print(f"System mode set to {args.mode}.")
            else:
                print(f"Failed to set system mode to {args.mode}.")
            
        elif args.command == 'diagnostics':
            cli.run_diagnostics()
            
        else:
            parser.print_help()
    
    except KeyboardInterrupt:
        print("\nOperation cancelled.")
    
    finally:
        # Cleanup
        cli.shutdown_system()
        cli.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()