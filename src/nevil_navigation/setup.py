from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'nevil_navigation'

setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name, f'{package_name}.nevil_navigation_api', f'{package_name}.nevil_navigation_api.examples'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'nevil_navigation_api'),
            [os.path.join('nevil_navigation', 'nevil_navigation_api', 'README.md')]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Navigation and movement control for Nevil-picar v2.0',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigation_node = nevil_navigation.navigation_node:main',
            'basic_movement = nevil_navigation.nevil_navigation_api.examples.basic_movement:main',
            'complex_navigation = nevil_navigation.nevil_navigation_api.examples.complex_navigation:main',
            'sensor_integration = nevil_navigation.nevil_navigation_api.examples.sensor_integration:main',
            'error_handling = nevil_navigation.nevil_navigation_api.examples.error_handling:main',
        ],
    },
)