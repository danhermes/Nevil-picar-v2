#!/usr/bin/env python3

import os
import sys
import argparse
import subprocess
import yaml
from ament_index_python.packages import get_package_share_directory

def main():
    parser = argparse.ArgumentParser(description='Run Nevil-picar v2.0 system tests')
    parser.add_argument('--scenario', type=str, default='all',
                        help='Specific scenario to test (all, end_to_end_navigation, voice_command_execution, error_handling)')
    parser.add_argument('--use-simulation', action='store_true', default=True,
                        help='Use simulation instead of real hardware')
    parser.add_argument('--verbose', action='store_true', help='Enable verbose output')
    
    args = parser.parse_args()
    
    # Get the package share directory
    nevil_testing_dir = get_package_share_directory('nevil_testing')
    
    # Load the test configuration
    config_file = os.path.join(nevil_testing_dir, 'config', 'test_config.yaml')
    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)
    
    # Set up environment variables
    env = os.environ.copy()
    env['NEVIL_USE_SIMULATION'] = 'true' if args.use_simulation else 'false'
    
    # Determine which scenarios to run
    if args.scenario == 'all':
        scenarios = [s['name'] for s in config['system_tests']['scenarios']]
    else:
        scenarios = [args.scenario]
    
    # Run the tests
    exit_code = 0
    
    for scenario in scenarios:
        print(f"Running system test scenario: {scenario}")
        
        # Find the scenario configuration
        scenario_config = next((s for s in config['system_tests']['scenarios'] if s['name'] == scenario), None)
        
        if scenario_config is None:
            print(f"  Error: Scenario {scenario} not found in configuration")
            exit_code = 1
            continue
        
        # Determine the test file
        test_file = f"test_{scenario}.py"
        test_path = os.path.join(nevil_testing_dir, 'test', 'system', test_file)
        
        if not os.path.exists(test_path):
            print(f"  Error: Test file {test_file} not found")
            exit_code = 1
            continue
        
        # Run the test using launch_test
        print(f"  Running {test_file}...")
        
        # Set the timeout
        timeout = scenario_config.get('timeout', 120.0)
        
        # Run the test
        cmd = ['launch_test', test_path, '--timeout', str(timeout)]
        
        # Add parameters from the configuration
        if 'parameters' in scenario_config:
            # Convert parameters to command line arguments
            for key, value in scenario_config['parameters'].items():
                if isinstance(value, dict):
                    # Handle nested parameters
                    for subkey, subvalue in value.items():
                        cmd.extend(['--param', f'{key}.{subkey}:={subvalue}'])
                else:
                    # Handle simple parameters
                    cmd.extend(['--param', f'{key}:={value}'])
        
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
    
    return exit_code

if __name__ == '__main__':
    sys.exit(main())