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
        # Include all launch files. This is the most important line here!
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ale_scar',
    maintainer_email='ale_scar@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'aruco_detector = camera_target_tracking.aruco_pose_estimator:main',
        "pose_filter = camera.pose_filt_cmd:main",
        ],
    },
)
