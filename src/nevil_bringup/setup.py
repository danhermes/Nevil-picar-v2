from setuptools import setup

package_name = 'nevil_bringup'

setup(
    name=package_name,
    version='2.0.0',
    packages=[package_name],
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
            'nevil_cli = nevil_bringup.cli:main',
        ],
    },
)