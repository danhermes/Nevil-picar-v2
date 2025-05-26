from setuptools import setup

package_name = 'nevil_realtime'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/rt_config.yaml']),
        ('share/' + package_name + '/launch', ['launch/nevil_realtime.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nevil Team',
    maintainer_email='user@example.com',
    description='Real-time components for Nevil-picar v2.0 using PREEMPT-RT Linux kernel',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rt_config_manager = nevil_realtime.rt_config_manager:main',
            'rt_sensor_node = nevil_realtime.rt_sensor_node:main',
            'rt_motor_control_node = nevil_realtime.rt_motor_control_node:main',
        ],
    },
)