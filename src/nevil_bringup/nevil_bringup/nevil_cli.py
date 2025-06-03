#!/usr/bin/env python3

import argparse
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class NevilCLI(Node):
    def __init__(self):
        super().__init__('nevil_cli')
        self.publisher_ = self.create_publisher(String, 'cli_command', 10)
        self.get_logger().info('Nevil CLI initialized')

    def send_command(self, command):
        msg = String()
        msg.data = command
        self.publisher_.publish(msg)
        self.get_logger().info(f'Sent command: {command}')

def main(args=None):
    parser = argparse.ArgumentParser(description='Nevil Command Line Interface')
    parser.add_argument('command', nargs='?', default='help', help='Command to execute')
    parser.add_argument('--param', help='Optional parameter for the command')
    parser.add_argument('--verbose', action='store_true', help='Enable verbose output')
    
    cli_args = parser.parse_args(args)
    
    if cli_args.command == 'help':
        print("Nevil CLI - Command Line Interface for Nevil Robot")
        print("\nAvailable commands:")
        print("  start       - Start the robot")
        print("  stop        - Stop the robot")
        print("  status      - Get the robot status")
        print("  navigate    - Navigate to a location (requires --param)")
        print("\nExamples:")
        print("  nevil_cli start")
        print("  nevil_cli navigate --param kitchen")
        return 0
    
    rclpy.init(args=args)
    cli = NevilCLI()
    
    try:
        if cli_args.verbose:
            cli.get_logger().info(f'Executing command: {cli_args.command}')
        
        command = cli_args.command
        if cli_args.param:
            command += f" {cli_args.param}"
        
        cli.send_command(command)
        
        # Wait a moment for the command to be processed
        rclpy.spin_once(cli, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        cli.destroy_node()
        rclpy.shutdown()
    
    return 0

if __name__ == '__main__':
    main()
