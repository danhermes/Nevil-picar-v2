#!/usr/bin/env python3

import os
import sys
import argparse
import subprocess
from ament_index_python.packages import get_package_share_directory

def main():
    parser = argparse.ArgumentParser(description='Run Nevil-picar v2.0 unit tests')
    parser.add_argument('--package', choices=['interfaces', 'core', 'navigation', 'perception', 'realtime', 'interfaces_ai', 'simulation', 'all'],
                        default='all', help='Package to test')
    parser.add_argument('--test-file', type=str, default='',
                        help='Specific test file to run (empty for all tests)')
    parser.add_argument('--use-simulation', action='store_true', default=True,
                        help='Use simulation instead of real hardware')
    parser.add_argument('--verbose', action='store_true', help='Enable verbose output')
    
    args = parser.parse_args()
    
    # Use the source directory instead of the install directory
    nevil_testing_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'src', 'nevil_testing')
    
    # Set up environment variables
    env = os.environ.copy()
    env['NEVIL_USE_SIMULATION'] = 'true' if args.use_simulation else 'false'
    
    # Determine which tests to run
    if args.package == 'all':
        packages = ['interfaces', 'interfaces_ai', 'core', 'navigation', 'perception', 'realtime', 'simulation']
    else:
        packages = [args.package]
    
    # Run the tests
    exit_code = 0
    
    for package in packages:
        print(f"Running unit tests for {package} package...")
        
        # Determine the test directory
        test_dir = os.path.join(nevil_testing_dir, 'test', 'unit', package)
        
        # Check if the directory exists
        if not os.path.exists(test_dir):
            print(f"  Warning: Test directory {test_dir} does not exist. Skipping...")
            continue
        
        # Determine which test files to run
        if args.test_file:
            test_files = [args.test_file]
        else:
            test_files = [f for f in os.listdir(test_dir) if f.startswith('test_') and f.endswith('.py')]
        
        # Run each test file
        for test_file in test_files:
            test_path = os.path.join(test_dir, test_file)
            print(f"  Running {test_file}...")
            
            # Run the test
            cmd = ['python3', test_path]
            
            if args.verbose:
                result = subprocess.run(cmd, env=env)
            else:
                result = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, env=env)
                
            if result.returncode != 0:
                print(f"  {test_file} failed with exit code {result.returncode}")
                if not args.verbose and result.stderr:
                    print(f"  Error output: {result.stderr.decode()}")
                exit_code = result.returncode
            else:
                print(f"  {test_file} passed")
    
    # Run C++ unit tests
    print("Running C++ unit tests...")
    
    # Determine which test targets to run
    if args.package == 'all':
        test_targets = ['test_system_manager_node', 'test_motion_control_node', 'test_camera_vision_node', 'test_rt_executor']
    elif args.package == 'core':
        test_targets = ['test_system_manager_node']
    elif args.package == 'navigation':
        test_targets = ['test_motion_control_node']
    elif args.package == 'perception':
        test_targets = ['test_camera_vision_node']
    elif args.package == 'realtime':
        test_targets = ['test_rt_executor']
    else:
        test_targets = []
    
    # Run each test target
    for target in test_targets:
        print(f"  Running {target}...")
        
        # Run the test
        cmd = ['colcon', 'test', '--packages-select', 'nevil_testing', '--ctest-args', '-R', f'^{target}$']
        
        if args.verbose:
            result = subprocess.run(cmd, env=env)
        else:
            result = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, env=env)
            
        if result.returncode != 0:
            print(f"  {target} failed with exit code {result.returncode}")
            if not args.verbose and result.stderr:
                print(f"  Error output: {result.stderr.decode()}")
            exit_code = result.returncode
        else:
            print(f"  {target} passed")
    
    return exit_code

if __name__ == '__main__':
    sys.exit(main())