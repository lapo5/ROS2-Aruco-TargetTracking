from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import os
import sys
import yaml
from launch.substitutions import EnvironmentVariable
import pathlib
import launch.actions
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

from launch.actions.execute_process import ExecuteProcess

def generate_launch_description():

    params = os.path.join(get_package_share_directory("camera_target_tracking"), 'params', 'params_pasqualone.yaml')
    
    for arg in sys.argv:
        if arg.startswith("project:="):
            project = arg.split(":=")[1]
            params = os.path.join(get_package_share_directory("camera_target_tracking"), 'params', 'params_' + project + '.yaml')


    return LaunchDescription([
        
        Node(
            package='camera_target_tracking',
            executable='aruco_detector',
            name='aruco_detector',
            output={
                    "stdout": "screen",
                    "stderr": "screen",
            },
            parameters=[params],
        )
])