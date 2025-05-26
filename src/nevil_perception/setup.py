from setuptools import setup
import os
from glob import glob

package_name = 'nevil_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Sensor processing and environment perception for Nevil-picar v2.0',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_vision = nevil_perception.camera_vision:main',
            'obstacle_detection = nevil_perception.obstacle_detection:main',
        ],
    },
)