#!/usr/bin/env python3
"""
State Fusion Launch File

Launches the state fusion and MAVROS bridge nodes.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare arguments
    enable_mavros_arg = DeclareLaunchArgument(
        'enable_mavros_bridge',
        default_value='true',
        description='Enable MAVROS bridge for PX4 integration'
    )

    vision_pose_rate_arg = DeclareLaunchArgument(
        'vision_pose_rate',
        default_value='30.0',
        description='Vision pose publishing rate to MAVROS (Hz)'
    )

    # Fusion Node
    fusion_node = Node(
        package='state_fusion',
        executable='fusion_node',
        name='fusion_node',
        output='screen',
        parameters=[{
            'vio_position_noise': 0.01,
            'vio_velocity_noise': 0.1,
            'satellite_position_noise': 5.0,
            'process_noise_position': 0.1,
            'process_noise_velocity': 0.5,
            'alignment_learning_rate': 0.1,
            'output_frame': 'map',
            'body_frame': 'base_link',
        }]
    )

    # MAVROS Bridge Node
    mavros_bridge_node = Node(
        package='state_fusion',
        executable='mavros_bridge_node',
        name='mavros_bridge',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_mavros_bridge')),
        parameters=[{
            'vision_pose_rate': LaunchConfiguration('vision_pose_rate'),
            'enable_setpoints': False,
            'local_frame': 'map',
        }]
    )

    return LaunchDescription([
        enable_mavros_arg,
        vision_pose_rate_arg,
        fusion_node,
        mavros_bridge_node,
    ])
