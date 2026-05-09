"""Launch all SOMARS controls nodes with shared parameters."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('somars_controls')
    params_file    = os.path.join(pkg_share, 'config', 'params.yaml')
    waypoints_file = os.path.join(pkg_share, 'config', 'waypoints.yaml')

    return LaunchDescription([
        Node(
            package='somars_controls',
            executable='offboard_manager',
            name='offboard_manager',
            parameters=[params_file],
            output='screen',
        ),
        Node(
            package='somars_controls',
            executable='guidance_node',
            name='guidance_node',
            parameters=[params_file, waypoints_file],
            output='screen',
        ),
        Node(
            package='somars_controls',
            executable='target_localizer',
            name='target_localizer',
            parameters=[params_file],
            output='screen',
        ),
    ])
