from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_target_tracking',
            namespace='cam_target_tracking',
            executable='aruco_detector',
            name='aruco_detector'
        ),
        Node(
            package='camera_target_tracking',
            namespace='cam_target_tracking',
            executable='ptu_to_robot_transform',
            name='ptu_to_robot_transform'
        ),
        Node(
            package='camera_target_tracking',
            namespace='cam_target_tracking',
            executable='camera_to_ptu_base',
            name='camera_to_ptu_base'
        )
    ])
