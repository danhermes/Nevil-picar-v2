from setuptools import setup
import os
from glob import glob

package_name = 'nevil_testing'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'test/unit'), glob('test/unit/**/*.py')),
        (os.path.join('share', package_name, 'test/integration'), glob('test/integration/**/*.py')),
        (os.path.join('share', package_name, 'test/system'), glob('test/system/**/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Testing framework for Nevil-picar v2.0',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'run_unit_tests = nevil_testing.test_runner:run_unit_tests',
            'run_integration_tests = nevil_testing.test_runner:run_integration_tests',
            'run_system_tests = nevil_testing.test_runner:run_system_tests',
            'run_all_tests = nevil_testing.test_runner:run_all_tests',
        ],
    },
)