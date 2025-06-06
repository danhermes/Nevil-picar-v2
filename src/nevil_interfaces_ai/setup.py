from setuptools import setup

package_name = 'nevil_interfaces_ai'

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
    maintainer_email='user@example.com',
    description='Text and voice interfaces for Nevil-picar v2.0',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'text_command_processor = nevil_interfaces_ai.text_command_processor:main',
            'speech_recognition_node = nevil_interfaces_ai.speech_recognition_node:main',
            'speech_synthesis_node = nevil_interfaces_ai.speech_synthesis_node:main',
            'dialog_manager_node = nevil_interfaces_ai.dialog_manager_node:main',
        ],
    },
)