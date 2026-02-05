#!/usr/bin/env python3
"""
Proxigo Scalence - Hardware Launch

Launches the system for real hardware (Orin Nano with Arducam).

Usage:
    ros2 launch proxigo_bringup hardware.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    # Arguments
    camera_device_arg = DeclareLaunchArgument(
        'camera_device',
        default_value='/dev/video0',
        description='Camera device path'
    )

    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url',
        default_value='/dev/ttyTHS1:921600',
        description='FCU connection URL'
    )

    satellite_data_path_arg = DeclareLaunchArgument(
        'satellite_data_path',
        default_value='/satellite_data',
        description='Path to satellite data'
    )

    region_id_arg = DeclareLaunchArgument(
        'region_id',
        default_value='',
        description='Satellite region ID (auto-detect if empty)'
    )

    # V4L2 Camera Node (for Arducam)
    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='arducam_camera',
        output='screen',
        parameters=[{
            'video_device': LaunchConfiguration('camera_device'),
            'image_size': [1920, 1080],
            'pixel_format': 'YUYV',
            'camera_frame_id': 'camera_optical_frame',
        }],
        remappings=[
            ('image_raw', '/vio/camera/image_raw'),
            ('camera_info', '/vio/camera/camera_info'),
        ]
    )

    # VIO Bridge (without simulation)
    vio_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('vio_bridge'),
                'launch',
                'vio_bridge.launch.py'
            ])
        ]),
        launch_arguments={
            'sim_mode': 'false',
        }.items()
    )

    # Satellite Matching
    satellite_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('satellite_matching'),
                'launch',
                'satellite_matching.launch.py'
            ])
        ]),
        launch_arguments={
            'satellite_data_path': LaunchConfiguration('satellite_data_path'),
            'region_id': LaunchConfiguration('region_id'),
        }.items()
    )

    # State Fusion
    fusion_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('state_fusion'),
                'launch',
                'state_fusion.launch.py'
            ])
        ]),
        launch_arguments={
            'enable_mavros_bridge': 'true',
        }.items()
    )

    # MAVROS
    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        name='mavros',
        output='screen',
        parameters=[{
            'fcu_url': LaunchConfiguration('fcu_url'),
            'gcs_url': 'udp://:14550@',
        }]
    )

    return LaunchDescription([
        camera_device_arg,
        fcu_url_arg,
        satellite_data_path_arg,
        region_id_arg,
        camera_node,
        vio_bridge_launch,
        satellite_launch,
        fusion_launch,
        mavros_node,
    ])
