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

    params_allied_1 = os.path.join(get_package_share_directory("hal_allied_vision_camera"), 'params', 'params_pasqualone.yaml')
    params_allied_2 = os.path.join(get_package_share_directory("hal_allied_vision_camera"), 'params', 'params_pasqualone.yaml')

    params_aruco = os.path.join(get_package_share_directory("camera_target_tracking"), 'params', 'params_pasqualone.yaml')
    params_aruco_filter = os.path.join(get_package_share_directory("aruco_pose_filter"), 'params', 'params_pasqualone.yaml')
    
    for arg in sys.argv:
        if arg.startswith("project:="):
            project = arg.split(":=")[1]
            params_allied_1 = os.path.join(get_package_share_directory("hal_allied_vision_camera"), 'params', 'params_' + project + '_cam1.yaml')
            params_allied_2 = os.path.join(get_package_share_directory("hal_allied_vision_camera"), 'params', 'params_' + project + '_cam2.yaml')
            params_aruco = os.path.join(get_package_share_directory("camera_target_tracking"), 'params', 'params_' + project + '_double_cam.yaml')
            params_aruco_filter = os.path.join(get_package_share_directory("aruco_pose_filter"), 'params', 'params_' + project + '.yaml')


    return LaunchDescription([
        Node(
            package='hal_allied_vision_camera',
            executable='av_node',
            name='av_node',
            parameters=[params_allied_1],
        ),
        Node(
            package='hal_allied_vision_camera',
            executable='av_node',
            name='av_node_2',
            parameters=[params_allied_2],
        ),
        Node(
            package='camera_target_tracking',
            executable='aruco_detector',
            name='aruco_detector',
            output={
                    "stdout": "screen",
                    "stderr": "screen",
            },
            parameters=[params_aruco],
        ),
        Node(
            package='aruco_pose_filter',
            executable='pose_filter',
            name='pose_filter',
            output={
                    "stdout": "screen",
                    "stderr": "screen",
            },
            parameters=[params_aruco_filter, {'marker_id': '69'}],
        ),
        Node(
            package='aruco_pose_filter',
            executable='pose_filter',
            name='pose_filter',
            output={
                    "stdout": "screen",
                    "stderr": "screen",
            },
            parameters=[params_aruco_filter, {'marker_id': '0'}],
        ),
        Node(
            package='aruco_pose_filter',
            executable='pose_filter',
            name='pose_filter',
            output={
                    "stdout": "screen",
                    "stderr": "screen",
            },
            parameters=[params_aruco_filter, {'marker_id': '1'}],
        ),
])
