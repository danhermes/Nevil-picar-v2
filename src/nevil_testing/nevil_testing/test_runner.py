#!/usr/bin/env python3

import os
import sys
import subprocess
import argparse
import rclpy
from rclpy.node import Node

class TestRunner(Node):
    """
    Test runner for Nevil-picar v2.0 testing framework.
    Provides functionality to run unit tests, integration tests, and system tests.
    """
    
    def __init__(self):
        super().__init__('test_runner')
        self.declare_parameter('test_type', 'all')
        self.declare_parameter('use_simulation', True)
        self.declare_parameter('verbose', False)
        
        self.test_type = self.get_parameter('test_type').value
        self.use_simulation = self.get_parameter('use_simulation').value
        self.verbose = self.get_parameter('verbose').value
        
        self.get_logger().info(f'Initialized test runner with test_type={self.test_type}, '
                              f'use_simulation={self.use_simulation}, verbose={self.verbose}')
    
    def run_tests(self):
        """Run the specified tests."""
        if self.test_type == 'unit':
            return self.run_unit_tests()
        elif self.test_type == 'integration':
            return self.run_integration_tests()
        elif self.test_type == 'system':
            return self.run_system_tests()
        elif self.test_type == 'all':
            return self.run_all_tests()
        else:
            self.get_logger().error(f'Unknown test type: {self.test_type}')
            return 1
    
    def run_unit_tests(self):
        """Run unit tests."""
        self.get_logger().info('Running unit tests...')
        
        # Run C++ unit tests
        cpp_result = self._run_command(['colcon', 'test', '--packages-select', 'nevil_testing', 
                                       '--ctest-args', '-R', '^test_.*$'])
        
        # Run Python unit tests
        py_result = self._run_command(['pytest', '-xvs', 
                                      os.path.join(self._get_package_share_directory(), 'test/unit')])
        
        return cpp_result or py_result
    
    def run_integration_tests(self):
        """Run integration tests."""
        self.get_logger().info('Running integration tests...')
        
        # Set up environment for integration tests
        env = os.environ.copy()
        env['NEVIL_USE_SIMULATION'] = 'true' if self.use_simulation else 'false'
        
        # Run integration tests
        result = self._run_command(['launch_test', 
                                   os.path.join(self._get_package_share_directory(), 
                                               'test/integration/test_navigation_system.py')], 
                                  env=env)
        
        return result
    
    def run_system_tests(self):
        """Run system tests."""
        self.get_logger().info('Running system tests...')
        
        # Set up environment for system tests
        env = os.environ.copy()
        env['NEVIL_USE_SIMULATION'] = 'true' if self.use_simulation else 'false'
        
        # Run system tests
        result = self._run_command(['launch_test', 
                                   os.path.join(self._get_package_share_directory(), 
                                               'test/system/test_end_to_end_navigation.py')], 
                                  env=env)
        
        return result
    
    def run_all_tests(self):
        """Run all tests."""
        self.get_logger().info('Running all tests...')
        
        unit_result = self.run_unit_tests()
        integration_result = self.run_integration_tests()
        system_result = self.run_system_tests()
        
        return unit_result or integration_result or system_result
    
    def _run_command(self, cmd, env=None):
        """Run a command and return the exit code."""
        self.get_logger().info(f'Running command: {" ".join(cmd)}')
        
        if self.verbose:
            result = subprocess.run(cmd, env=env)
        else:
            result = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, env=env)
            
        if result.returncode != 0:
            self.get_logger().error(f'Command failed with exit code {result.returncode}')
            if not self.verbose and result.stderr:
                self.get_logger().error(f'Error output: {result.stderr.decode()}')
        
        return result.returncode
    
    def _get_package_share_directory(self):
        """Get the package share directory."""
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory('nevil_testing')


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    test_runner = TestRunner()
    exit_code = test_runner.run_tests()
    
    test_runner.destroy_node()
    rclpy.shutdown()
    
    return exit_code


def run_unit_tests():
    """Entry point for running unit tests."""
    sys.argv.extend(['--ros-args', '-p', 'test_type:=unit'])
    return main()


def run_integration_tests():
    """Entry point for running integration tests."""
    sys.argv.extend(['--ros-args', '-p', 'test_type:=integration'])
    return main()


def run_system_tests():
    """Entry point for running system tests."""
    sys.argv.extend(['--ros-args', '-p', 'test_type:=system'])
    return main()


def run_all_tests():
    """Entry point for running all tests."""
    sys.argv.extend(['--ros-args', '-p', 'test_type:=all'])
    return main()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Nevil-picar v2.0 test runner')
    parser.add_argument('--test-type', choices=['unit', 'integration', 'system', 'all'], 
                        default='all', help='Type of tests to run')
    parser.add_argument('--use-simulation', action='store_true', default=True,
                        help='Use simulation instead of real hardware')
    parser.add_argument('--verbose', action='store_true', help='Enable verbose output')
    
    args = parser.parse_args()
    
    sys.argv.extend([
        '--ros-args',
        '-p', f'test_type:={args.test_type}',
        '-p', f'use_simulation:={str(args.use_simulation).lower()}',
        '-p', f'verbose:={str(args.verbose).lower()}'
    ])
    
    sys.exit(main())