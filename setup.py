from setuptools import setup
import os
from glob import glob

package_name = 'sensor_sync'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # firmware 폴더도 설치
        (os.path.join('share', package_name, 'firmware'),
            glob('firmware/*')),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Multi-Sensor Synchronization System (Teensy + ROS2)',
    license='MIT',
    entry_points={
        'console_scripts': [
            'sync_node = sensor_sync.sync_node:main',
        ],
    },
)
