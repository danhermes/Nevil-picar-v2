#!/usr/bin/env python3

import os
import sys
import subprocess
import argparse

def main():
    parser = argparse.ArgumentParser(description='Run Nevil-picar v2.0 interfaces_ai tests')
    parser.add_argument('--test-file', type=str, default='',
                        help='Specific test file to run (empty for all tests)')
    parser.add_argument('--verbose', action='store_true', help='Enable verbose output')
    
    args = parser.parse_args()
    
    # Use the source directory
    nevil_testing_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), 'nevil_testing')
    
    # Set up environment variables
    env = os.environ.copy()
    
    # Add the source directory to PYTHONPATH
    if 'PYTHONPATH' in env:
        env['PYTHONPATH'] = os.path.dirname(nevil_testing_dir) + ':' + env['PYTHONPATH']
    else:
        env['PYTHONPATH'] = os.path.dirname(nevil_testing_dir)
    
    # Determine the test directory
    test_dir = os.path.join(nevil_testing_dir, 'test', 'unit', 'interfaces_ai')
    
    # Check if the directory exists
    if not os.path.exists(test_dir):
        print(f"Error: Test directory {test_dir} does not exist.")
        return 1
    
    # Determine which test files to run
    if args.test_file:
        test_files = [args.test_file]
    else:
        test_files = [f for f in os.listdir(test_dir) if f.startswith('test_') and f.endswith('.py')]
    
    # Run each test file
    exit_code = 0
    for test_file in test_files:
        test_path = os.path.join(test_dir, test_file)
        print(f"Running {test_file}...")
        
        # Run the test
        cmd = ['python3', test_path]
        
        if args.verbose:
            result = subprocess.run(cmd, env=env)
        else:
            result = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, env=env)
            
        if result.returncode != 0:
            print(f"{test_file} failed with exit code {result.returncode}")
            if not args.verbose and result.stderr:
                print(f"Error output: {result.stderr.decode()}")
            exit_code = result.returncode
        else:
            print(f"{test_file} passed")
    
    return exit_code

if __name__ == '__main__':
    sys.exit(main())