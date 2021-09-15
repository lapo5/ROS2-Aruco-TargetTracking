from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hal_allied_vision_camera',
            namespace='sensors',
            executable='av_node',
            name='hal_allied_vision_camera'
        ),
        Node(
            package='hal_flir_d46',
            namespace='sensors',
            executable='HALFlirD46',
            name='hal_flir_d46'
        ),
        Node(
            package='camera_target_tracking',
            namespace='target_tracking',
            executable='aruco_detector',
            name='aruco_detector'
        ),
        Node(
            package='camera_target_tracking',
            namespace='target_tracking',
            executable='camera_to_ptu_base',
            name='camera_to_ptu_base'
        ),
        Node(
            package='pasqua_tf',
            namespace='tf_tree',
            executable='tf_tree_rear_camera',
            name='tf_tree_rear_cam_pasqua',
            output='screen',
        ),
    ])
