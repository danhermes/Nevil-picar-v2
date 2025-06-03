from setuptools import setup

package_name = 'nevil_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Perception system for Nevil-picar v2.0',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'perception_node = nevil_perception.perception_node:main',
            'camera_vision.py = nevil_perception.camera_vision:main',
            'obstacle_detection.py = nevil_perception.obstacle_detection:main',
        ],
    },
)
