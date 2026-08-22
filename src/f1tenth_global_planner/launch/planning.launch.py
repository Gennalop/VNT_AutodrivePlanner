import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('f1tenth_global_planner')
    map_yaml = os.path.join(pkg_share, 'maps', 'Autodrive_DefaultMap_obs.yaml')
    params_yaml = os.path.join(pkg_share, 'config', 'planning_params.yaml')
    rviz_cfg = os.path.join(pkg_share, 'config', 'planning.rviz')

    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_yaml, 'frame_id': 'map'}],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[{'autostart': True, 'node_names': ['map_server']}],
        ),
        Node(
            package='f1tenth_global_planner',
            executable='planner_node',
            name='lpa_star_planner',
            output='screen',
            parameters=[params_yaml],
        ),
        Node(
            package='f1tenth_global_planner',
            executable='smoothing_node',
            name='path_smoother',
            output='screen',
            parameters=[params_yaml],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_cfg],
        ),
    ])