# ~/autodrive_ws/src/f1tenth_lqr_control/launch/control.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='f1tenth_lqr_control',
            executable='lqr_controller_node',
            name='lqr_controller',
            output='screen',
            parameters=[{
                'target_speed_mps': 1.0,   # sube esto para la Prueba 2, una vez estable en Prueba 1
                'throttle_limit': 0.4,
            }],
        ),
    ])