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

    rviz_config_path = os.path.join(get_package_share_directory("camera_target_tracking"), 'rviz', 'rviz_pasqualone.rviz')
    camera_link = 'parking_camera_link'
    for arg in sys.argv:
        if arg.startswith("project:="):
            project = arg.split(":=")[1]
            rviz_config_path = os.path.join(get_package_share_directory("camera_target_tracking"), 'rviz', 'rviz_' + project + '.rviz')
            if project == 'xarm':
                camera_link = 'wrist_camera_link'
            elif project == 'pasqualone':
                camera_link = 'parking_camera_link'
            elif project == 'xarm_wac':
                camera_link = 'wac_camera_link'
    return LaunchDescription([
        
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_for_target_tracking',
            arguments=['-d', rviz_config_path]
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            output='screen',
            arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'world', camera_link]
        )
])
