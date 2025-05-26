#!/usr/bin/env python3

import os
import sys
import yaml
import argparse
import subprocess
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rcl_interfaces.msg import SetParametersResult


class RTConfigManager(Node):
    """
    Real-time configuration manager for Nevil-picar v2.0.
    
    This node manages real-time parameters and provides a ROS2 interface
    for configuring real-time features.
    """
    
    def __init__(self):
        super().__init__('rt_config_manager')
        
        # Get the path to the config file
        self.config_path = os.path.join(
            get_package_share_directory('nevil_realtime'),
            'config',
            'rt_config.yaml'
        )
        
        # Load the configuration
        self.config = self.load_config()
        
        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('global.enable_realtime', self.config['global']['enable_realtime'],
                 ParameterDescriptor(type=ParameterType.PARAMETER_BOOL,
                                    description='Enable real-time features')),
                ('global.lock_memory', self.config['global']['lock_memory'],
                 ParameterDescriptor(type=ParameterType.PARAMETER_BOOL,
                                    description='Lock memory to prevent paging')),
                ('global.isolated_core', self.config['global']['isolated_core'],
                 ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER,
                                    description='CPU core to isolate for real-time tasks')),
                ('global.max_latency_us', self.config['global']['max_latency_us'],
                 ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER,
                                    description='Maximum latency allowed for critical operations')),
            ]
        )
        
        # Declare parameters for thread priorities
        for node_name, priority in self.config['priorities'].items():
            self.declare_parameter(
                f'priorities.{node_name}',
                priority,
                ParameterDescriptor(
                    type=ParameterType.PARAMETER_INTEGER,
                    description=f'Thread priority for {node_name} node'
                )
            )
        
        # Set up parameter callback
        self.add_on_set_parameters_callback(self.parameters_callback)
        
        # Create a timer to periodically check real-time status
        self.create_timer(10.0, self.check_rt_status)
        
        self.get_logger().info('Real-time configuration manager initialized')
        
        # Apply initial configuration
        if self.config['global']['enable_realtime']:
            self.apply_rt_configuration()
    
    def load_config(self):
        """Load configuration from YAML file."""
        try:
            with open(self.config_path, 'r') as f:
                return yaml.safe_load(f)
        except Exception as e:
            self.get_logger().error(f'Failed to load configuration: {e}')
            # Return default configuration
            return {
                'global': {
                    'enable_realtime': False,
                    'lock_memory': False,
                    'isolated_core': -1,
                    'max_latency_us': 1000
                },
                'priorities': {
                    'motion_control': 90,
                    'obstacle_avoidance': 85,
                    'sensor_processing': 80,
                    'navigation': 75,
                    'camera_vision': 70,
                    'voice_control': 60,
                    'ai_processing': 50,
                    'system_manager': 40
                },
                'cpu_affinity': {},
                'communication': {
                    'use_cyclone_dds': True,
                    'qos': {}
                },
                'hardware': {
                    'polling_rates': {},
                    'mutex_timeout_ms': 100,
                    'enable_watchdog': True,
                    'watchdog_timeout_ms': 500
                },
                'monitoring': {
                    'enable_latency_monitoring': True,
                    'latency_report_interval': 10,
                    'log_level': 'INFO',
                    'enable_tracing': False
                }
            }
    
    def save_config(self):
        """Save configuration to YAML file."""
        try:
            with open(self.config_path, 'w') as f:
                yaml.dump(self.config, f, default_flow_style=False)
            self.get_logger().info('Configuration saved')
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to save configuration: {e}')
            return False
    
    def parameters_callback(self, params):
        """Callback for parameter changes."""
        for param in params:
            # Handle global parameters
            if param.name.startswith('global.'):
                param_name = param.name.split('.')[1]
                self.config['global'][param_name] = param.value
            
            # Handle priority parameters
            elif param.name.startswith('priorities.'):
                node_name = param.name.split('.')[1]
                self.config['priorities'][node_name] = param.value
        
        # Save the updated configuration
        self.save_config()
        
        # Apply the new configuration if real-time is enabled
        if self.config['global']['enable_realtime']:
            self.apply_rt_configuration()
        
        return SetParametersResult(successful=True)
    
    def apply_rt_configuration(self):
        """Apply real-time configuration to the system."""
        self.get_logger().info('Applying real-time configuration')
        
        # Check if we're running as root
        if os.geteuid() != 0:
            self.get_logger().warn('Not running as root, some real-time settings may not be applied')
        
        # Lock memory if enabled
        if self.config['global']['lock_memory']:
            try:
                # This is a Python wrapper around mlockall()
                import ctypes
                libc = ctypes.CDLL('libc.so.6')
                MCL_CURRENT = 1
                MCL_FUTURE = 2
                result = libc.mlockall(MCL_CURRENT | MCL_FUTURE)
                if result != 0:
                    self.get_logger().error('Failed to lock memory')
                else:
                    self.get_logger().info('Memory locked successfully')
            except Exception as e:
                self.get_logger().error(f'Failed to lock memory: {e}')
        
        # Set up isolated core if enabled
        if self.config['global']['isolated_core'] >= 0:
            try:
                core = self.config['global']['isolated_core']
                # Check if isolcpus is already set in kernel parameters
                with open('/proc/cmdline', 'r') as f:
                    cmdline = f.read()
                
                if f'isolcpus={core}' not in cmdline:
                    self.get_logger().warn(
                        f'CPU core {core} is not isolated. Add isolcpus={core} '
                        f'nohz_full={core} rcu_nocbs={core} to kernel parameters '
                        'in /boot/cmdline.txt and reboot.'
                    )
            except Exception as e:
                self.get_logger().error(f'Failed to check isolated core: {e}')
        
        # Check if we're running on a PREEMPT-RT kernel
        self.check_rt_kernel()
    
    def check_rt_kernel(self):
        """Check if we're running on a PREEMPT-RT kernel."""
        try:
            result = subprocess.run(['uname', '-a'], capture_output=True, text=True)
            kernel_info = result.stdout
            
            if 'PREEMPT' in kernel_info:
                self.get_logger().info('Running on a PREEMPT-RT kernel')
                return True
            else:
                self.get_logger().warn(
                    'Not running on a PREEMPT-RT kernel. '
                    'Real-time performance will be limited.'
                )
                return False
        except Exception as e:
            self.get_logger().error(f'Failed to check kernel: {e}')
            return False
    
    def check_rt_status(self):
        """Periodically check real-time status."""
        if not self.config['global']['enable_realtime']:
            return
        
        # Check if we're running on a PREEMPT-RT kernel
        is_rt_kernel = self.check_rt_kernel()
        
        # Check if memory is locked
        try:
            # This is a simplified check, not 100% accurate
            with open('/proc/self/status', 'r') as f:
                status = f.read()
                if 'VmLck' in status and 'kB' in status.split('VmLck:')[1].split('\n')[0]:
                    self.get_logger().info('Memory is locked')
                else:
                    self.get_logger().warn('Memory is not locked')
        except Exception as e:
            self.get_logger().error(f'Failed to check memory lock status: {e}')
        
        # Check thread priorities
        try:
            result = subprocess.run(['ps', '-eLo', 'pid,rtprio,cmd'], capture_output=True, text=True)
            ps_output = result.stdout
            
            # Look for ROS2 nodes
            ros2_lines = [line for line in ps_output.split('\n') if 'ros2' in line]
            
            if ros2_lines:
                self.get_logger().info('ROS2 nodes with real-time priority:')
                for line in ros2_lines:
                    if line.split()[1] != '-':  # Has real-time priority
                        self.get_logger().info(line)
            else:
                self.get_logger().warn('No ROS2 nodes with real-time priority found')
        except Exception as e:
            self.get_logger().error(f'Failed to check thread priorities: {e}')
    
    def edit_config(self):
        """Open the configuration file in an editor."""
        editor = os.environ.get('EDITOR', 'nano')
        try:
            subprocess.run([editor, self.config_path])
            # Reload the configuration after editing
            self.config = self.load_config()
            self.get_logger().info('Configuration reloaded')
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to edit configuration: {e}')
            return False


def main(args=None):
    """Main function."""
    rclpy.init(args=args)
    
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description='Real-time configuration manager')
    parser.add_argument('--edit', action='store_true', help='Edit the configuration file')
    parser.add_argument('--apply', action='store_true', help='Apply the configuration')
    parser.add_argument('--save', action='store_true', help='Save the configuration')
    parser.add_argument('--check', action='store_true', help='Check real-time status')
    
    # If no arguments are provided, run as a ROS2 node
    if len(sys.argv) == 1:
        node = RTConfigManager()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
        return
    
    # Otherwise, handle command-line arguments
    args = parser.parse_args()
    
    node = RTConfigManager()
    
    if args.edit:
        node.edit_config()
    
    if args.apply:
        node.apply_rt_configuration()
    
    if args.save:
        node.save_config()
    
    if args.check:
        node.check_rt_kernel()
        node.check_rt_status()
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()