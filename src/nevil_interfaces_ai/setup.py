from setuptools import setup, find_packages
import os
from glob import glob
from setuptools import setup

package_name = 'nevil_interfaces_ai'

import numpy
print(">>> NumPy version seen by setup.py:", numpy.__version__)
print(">>> NumPy include dir:", numpy.get_include())


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=[package_name, f"{package_name}.*"]),
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*.launch.py'))),
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
            'ai_interface_node = nevil_interfaces_ai.ai_interface_node:main',
        ],
    },
)
