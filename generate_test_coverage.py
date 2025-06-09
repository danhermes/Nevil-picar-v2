#!/usr/bin/env python3

import os
import sys
import subprocess
import argparse
import json
from datetime import datetime

def main():
    parser = argparse.ArgumentParser(description='Generate test coverage report for Nevil-picar v2.0')
    parser.add_argument('--output-dir', type=str, default='docs/tests/coverage',
                        help='Directory to store coverage reports')
    parser.add_argument('--package', choices=['core', 'navigation', 'perception', 'realtime', 'interfaces_ai', 'simulation', 'all'], 
                        default='all', help='Package to generate coverage for')
    parser.add_argument('--verbose', action='store_true', help='Enable verbose output')
    
    args = parser.parse_args()
    
    # Get the base directory
    base_dir = os.path.dirname(os.path.abspath(__file__))
    
    # Create output directory if it doesn't exist
    output_dir = os.path.join(base_dir, args.output_dir)
    os.makedirs(output_dir, exist_ok=True)
    
    # Set up environment variables
    env = os.environ.copy()
    
    # Add the source directory to PYTHONPATH
    src_dir = os.path.join(base_dir, 'src')
    if 'PYTHONPATH' in env:
        env['PYTHONPATH'] = f"{src_dir}:{env['PYTHONPATH']}"
    else:
        env['PYTHONPATH'] = src_dir
    
    # Determine which packages to generate coverage for
    if args.package == 'all':
        packages = ['core', 'navigation', 'perception', 'realtime', 'interfaces_ai', 'simulation']
    else:
        packages = [args.package]
    
    # Generate coverage report for each package
    coverage_data = {}
    
    for package in packages:
        print(f"Generating coverage report for {package} package...")
        
        # Determine the package directory
        package_dir = os.path.join(src_dir, f"nevil_{package}")
        
        if not os.path.exists(package_dir):
            print(f"  Warning: Package directory {package_dir} does not exist. Skipping...")
            continue
        
        # Determine the test directory
        test_dir = os.path.join(src_dir, 'nevil_testing', 'test', 'unit', package)
        
        if not os.path.exists(test_dir):
            print(f"  Warning: Test directory {test_dir} does not exist. Skipping...")
            continue
        
        # Determine which test files to run
        test_files = [f for f in os.listdir(test_dir) if f.startswith('test_') and f.endswith('.py')]
        
        if not test_files:
            print(f"  Warning: No test files found in {test_dir}. Skipping...")
            continue
        
        # Create a coverage configuration file
        coverage_rc = os.path.join(base_dir, '.coveragerc')
        with open(coverage_rc, 'w') as f:
            f.write(f"""[run]
source = {package_dir}
omit = */test/*

[report]
exclude_lines =
    pragma: no cover
    def __repr__
    raise NotImplementedError
    if __name__ == .__main__.:
    pass
    raise ImportError
""")
        
        # Run coverage for each test file
        for test_file in test_files:
            test_path = os.path.join(test_dir, test_file)
            print(f"  Running coverage for {test_file}...")
            
            # Create a wrapper script that adds the necessary imports and runs coverage
            wrapper_script = os.path.join(base_dir, f"wrapper_coverage_{test_file}")
            
            # Special handling for the realtime test that has import issues
            if package == 'realtime' and test_file == 'test_rt_executor.py':
                with open(wrapper_script, 'w') as f:
                    f.write(f"""#!/usr/bin/env python3
import os
import sys
import coverage

# Add the source directory to the Python path
sys.path.insert(0, '{src_dir}')

# Create a symbolic link for nevil_testing module
import importlib.util
if importlib.util.find_spec('nevil_testing') is None:
    # Create a symbolic link in the Python path
    nevil_testing_dir = os.path.join('{src_dir}', 'nevil_testing', 'nevil_testing')
    sys.path.append(nevil_testing_dir)
    
    # Also add the parent directory to handle relative imports
    sys.path.append(os.path.dirname(nevil_testing_dir))

# Mock the missing RealtimeCallbackGroup
import rclpy.callback_groups
if not hasattr(rclpy.callback_groups, 'RealtimeCallbackGroup'):
    class RealtimeCallbackGroup:
        pass
    rclpy.callback_groups.RealtimeCallbackGroup = RealtimeCallbackGroup

# Start coverage
cov = coverage.Coverage(config_file='{coverage_rc}')
cov.start()

# Run the original test
try:
    exec(open('{test_path}').read())
finally:
    # Stop coverage
    cov.stop()
    cov.save()
""")
            else:
                # For other tests, create a simple wrapper that adds the Python path
                with open(wrapper_script, 'w') as f:
                    f.write(f"""#!/usr/bin/env python3
import os
import sys
import coverage

# Add the source directory to the Python path
sys.path.insert(0, '{src_dir}')

# Create a symbolic link for nevil_testing module
import importlib.util
if importlib.util.find_spec('nevil_testing') is None:
    # Create a symbolic link in the Python path
    nevil_testing_dir = os.path.join('{src_dir}', 'nevil_testing', 'nevil_testing')
    sys.path.append(nevil_testing_dir)
    
    # Also add the parent directory to handle relative imports
    sys.path.append(os.path.dirname(nevil_testing_dir))

# Start coverage
cov = coverage.Coverage(config_file='{coverage_rc}')
cov.start()

# Run the original test
try:
    exec(open('{test_path}').read())
finally:
    # Stop coverage
    cov.stop()
    cov.save()
""")
            
            os.chmod(wrapper_script, 0o755)
            
            # Run the wrapper script
            cmd = [wrapper_script]
            
            try:
                if args.verbose:
                    result = subprocess.run(cmd, env=env)
                else:
                    result = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, env=env)
                    
                if result.returncode != 0:
                    print(f"  Warning: Coverage run failed for {test_file} with exit code {result.returncode}")
                    if not args.verbose and result.stderr:
                        print(f"  Error output: {result.stderr.decode()}")
                    continue
            except Exception as e:
                print(f"  Error running coverage for {test_file}: {e}")
                continue
            finally:
                # Clean up the wrapper script
                os.remove(wrapper_script)
        
        # Generate XML report
        coverage_output = os.path.join(output_dir, f"{package}_coverage.xml")
        cmd = ['python3', '-m', 'coverage', 'xml', '-o', coverage_output]
        
        try:
            if args.verbose:
                result = subprocess.run(cmd, env=env)
            else:
                result = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, env=env)
                
            if result.returncode != 0:
                print(f"  Warning: Coverage XML generation failed for {package} with exit code {result.returncode}")
                if not args.verbose and result.stderr:
                    print(f"  Error output: {result.stderr.decode()}")
                continue
        except Exception as e:
            print(f"  Error generating XML report for {package}: {e}")
            continue
        
        # Generate HTML report
        html_output = os.path.join(output_dir, package)
        os.makedirs(html_output, exist_ok=True)
        
        cmd = ['python3', '-m', 'coverage', 'html', '-d', html_output]
        
        try:
            if args.verbose:
                result = subprocess.run(cmd, env=env)
            else:
                result = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, env=env)
                
            if result.returncode != 0:
                print(f"  Warning: Coverage HTML generation failed for {package} with exit code {result.returncode}")
                if not args.verbose and result.stderr:
                    print(f"  Error output: {result.stderr.decode()}")
                continue
        except Exception as e:
            print(f"  Error generating HTML report for {package}: {e}")
            continue
        
        # Get coverage report
        cmd = ['python3', '-m', 'coverage', 'report']
        
        try:
            result = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, env=env)
            
            if result.returncode != 0:
                print(f"  Warning: Coverage report failed for {package} with exit code {result.returncode}")
                if result.stderr:
                    print(f"  Error output: {result.stderr.decode()}")
                continue
            
            # Parse coverage report
            report_lines = result.stdout.decode().strip().split('\n')
            
            # Extract total coverage
            total_line = report_lines[-1] if report_lines else "TOTAL 0 0 0 0 0%"
            total_coverage = total_line.split()[-1].strip('%')
            
            try:
                coverage_value = float(total_coverage)
            except ValueError:
                coverage_value = 0.0
            
            coverage_data[package] = {
                'total': coverage_value,
                'timestamp': datetime.now().isoformat(),
                'report_path': os.path.relpath(html_output, base_dir)
            }
            
            print(f"  Coverage for {package}: {total_coverage}%")
        except Exception as e:
            print(f"  Error parsing coverage report for {package}: {e}")
            continue
    
    # Generate summary report
    summary_path = os.path.join(output_dir, 'coverage_summary.json')
    
    with open(summary_path, 'w') as f:
        json.dump(coverage_data, f, indent=2)
    
    # Generate Markdown report
    markdown_path = os.path.join(output_dir, 'COVERAGE.md')
    
    with open(markdown_path, 'w') as f:
        f.write("# Nevil-picar-v2 Test Coverage Report\n\n")
        f.write(f"Generated on: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n\n")
        
        f.write("## Coverage Summary\n\n")
        f.write("| Package | Coverage |\n")
        f.write("|---------|----------|\n")
        
        for package, data in coverage_data.items():
            f.write(f"| nevil_{package} | {data['total']}% |\n")
        
        f.write("\n## Details\n\n")
        
        for package, data in coverage_data.items():
            f.write(f"### nevil_{package}\n\n")
            f.write(f"- Total Coverage: {data['total']}%\n")
            f.write(f"- Report: [{package} coverage report]({data['report_path']}/index.html)\n")
            f.write(f"- Generated: {data['timestamp']}\n\n")
    
    print(f"\nCoverage reports generated in {output_dir}")
    print(f"Summary report: {os.path.relpath(markdown_path, base_dir)}")
    
    # Clean up
    if os.path.exists(coverage_rc):
        os.remove(coverage_rc)

if __name__ == '__main__':
    main()