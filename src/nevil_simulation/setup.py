from setuptools import setup
import os
from glob import glob

package_name = 'nevil_simulation'

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
        (os.path.join('share', package_name, 'models'), glob('models/**/*', recursive=True)),
        (os.path.join('share', package_name, 'environments'), glob('environments/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nevil Team',
    maintainer_email='maintainer@example.com',
    description='Digital twin simulation for Nevil-picar v2.0',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simulation_manager = nevil_simulation.simulation_manager:main',
            'environment_generator = nevil_simulation.environment_generator:main',
            'sensor_simulator = nevil_simulation.sensor_simulator:main',
            'physics_simulator = nevil_simulation.physics_simulator:main',
            'visualization_bridge = nevil_simulation.visualization_bridge:main',
        ],
    },
)