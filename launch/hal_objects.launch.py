from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='allied_vision_camera',
            namespace='camera',
            executable='av_node',
            name='hal_allied_vision_camera'
        ),
        Node(
            package='hal_ptu_flir_d46',
            namespace='ptu',
            executable='HALPTUFlirD46',
            name='hal_flir_ptu'
        ),
        Node(
            package='hal_xsens_mti_28a',
            namespace='imu',
            executable='imu_node',
            name='hal_xsens_imu'
        )
    ])
