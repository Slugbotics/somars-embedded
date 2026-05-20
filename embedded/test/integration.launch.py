"""Launch mock nodes + all controls nodes for hardware-free integration testing.

Usage:
  ros2 launch somars_controls integration.launch.py

Then in another terminal:
  ros2 topic echo /targets/ned
  ros2 topic echo /fmu/in/trajectory_setpoint
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('somars_controls')
    params_file = os.path.join(pkg_share, 'config', 'params.yaml')

    return LaunchDescription([
        # Mock PX4 + vision publisher
        Node(
            package='somars_controls',
            executable='mock_nodes',
            name='mock_nodes',
            parameters=[{
                'altitude_m': 20.0,
                'north_m': 100.0,
                'east_m': 50.0,
                'yaw_deg': 0.0,
                'detection_u': 400.0,   # offset right of center → East
                'detection_v': 300.0,   # offset below center → North
                'detection_class': 0,   # red target
                'publish_rate_hz': 10.0,
                'enable_detections': True,
            }],
            output='screen',
        ),

        # Real controls nodes
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
            parameters=[params_file],
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
