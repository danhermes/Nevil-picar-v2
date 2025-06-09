from setuptools import setup
import os
from glob import glob

package_name = 'nevil_interfaces_ai'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*.launch.py'))),
        # Include scripts directory
        (os.path.join('share', package_name, 'scripts'),
         glob(os.path.join('scripts', '*.py'))),
    ],
    install_requires=[
        'setuptools',
        'python-dotenv',
    ],
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
            'audio_hardware_interface_test = nevil_interfaces_ai.audio_hardware_interface:main',
            'test_audio_hardware = nevil_interfaces_ai.test_audio_hardware:main',
            'test_env_loading = nevil_interfaces_ai.test_env_loading:main',
            'ai_interface_node = scripts.ai_interface_node:main',
        ],
    },
)