from setuptools import setup

package_name = 'nevil_bringup'

setup(
    name=package_name,
    version='2.0.0',
    packages=[package_name, 'scripts'],  # Add scripts directory as a package
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nevil Team',
    maintainer_email='maintainer@example.com',
    description='Integration package for the Nevil-picar v2.0 project',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nevil_cli = nevil_bringup.nevil_cli:main',
            'system_monitor = scripts.system_monitor:main',  # Add system_monitor entry point
        ],
    },
    # Include directories in the package
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/common.launch.py',
             'launch/development.launch.py',
             'launch/full_system.launch.py',
             'launch/integration_test.launch.py',
             'launch/minimal_system.launch.py',
             'launch/physical_robot.launch.py',
             'launch/simulation.launch.py']),
        ('share/' + package_name + '/config',
            ['config/default_config.yaml',
             'config/development_config.yaml',
             'config/minimal_config.yaml',
             'config/physical_robot_config.yaml',
             'config/simulation_config.yaml']),
        ('share/' + package_name + '/scripts',
            ['scripts/system_monitor.py',
             'scripts/battery_monitor.py',
             'scripts/hardware_init.py',
             'scripts/nevil_cli.py',
             'scripts/parameter_tuning_ui.py',
             'scripts/system_monitor_wrapper']),
    ],
)