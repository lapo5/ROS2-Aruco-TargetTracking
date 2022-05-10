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

def generate_launch_description():

    params_allied_2 = os.path.join(get_package_share_directory("hal_allied_vision_camera"), 'params', 'params_xarm_cam2.yaml')


    return LaunchDescription([
        Node(
            package='hal_allied_vision_camera',
            executable='av_node',
            name='av_node_2',
            parameters=[params_allied_2],
        ),
])
