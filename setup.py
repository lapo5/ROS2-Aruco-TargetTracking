from setuptools import setup
import os

from glob import glob
from setuptools import setup

package_name = 'camera_target_tracking'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'params'), glob('params/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Marco Lapolla',
    maintainer_email='marco.lapolla5@gmail.com',
    description='Aruco Target Tracking',
    license='BSD',
    entry_points={
        'console_scripts': [
        'aruco_detector = camera_target_tracking.aruco_detector:main',
        ],
    },
)
