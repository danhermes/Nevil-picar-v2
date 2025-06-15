from setuptools import setup
import os
from glob import glob

package_name = 'nevil_core'

import numpy
print(">>> NumPy version seen by setup.py:", numpy.__version__)
print(">>> NumPy include dir:", numpy.get_include())

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Core package for Nevil robot',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'setup_tools = nevil_core.setup_tools:main',
        ],
    },
)