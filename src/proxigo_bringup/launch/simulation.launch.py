#!/usr/bin/env python3
"""
Proxigo Scalence - Full Simulation Launch

Launches the complete simulation stack:
- VIO Bridge with camera/IMU simulators
- Satellite Matching
- State Fusion
- MAVROS Bridge

Usage:
    ros2 launch proxigo_bringup simulation.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    # Arguments
    satellite_data_path_arg = DeclareLaunchArgument(
        'satellite_data_path',
        default_value='/satellite_data',
        description='Path to satellite data'
    )

    region_id_arg = DeclareLaunchArgument(
        'region_id',
        default_value='test_region',
        description='Satellite region ID'
    )

    initial_lat_arg = DeclareLaunchArgument(
        'initial_lat',
        default_value='39.678123',
        description='Initial latitude'
    )

    initial_lon_arg = DeclareLaunchArgument(
        'initial_lon',
        default_value='-75.750456',
        description='Initial longitude'
    )

    # Include VIO Bridge launch
    vio_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('vio_bridge'),
                'launch',
                'vio_bridge.launch.py'
            ])
        ]),
        launch_arguments={
            'sim_mode': 'true',
            'satellite_image_path': PathJoinSubstitution([
                LaunchConfiguration('satellite_data_path'),
                'regions',
                LaunchConfiguration('region_id'),
                'satellite_synthetic.png'
            ]),
        }.items()
    )

    # Include Satellite Matching launch
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
            'initial_lat': LaunchConfiguration('initial_lat'),
            'initial_lon': LaunchConfiguration('initial_lon'),
        }.items()
    )

    # Include State Fusion launch
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

    return LaunchDescription([
        satellite_data_path_arg,
        region_id_arg,
        initial_lat_arg,
        initial_lon_arg,
        vio_bridge_launch,
        satellite_launch,
        fusion_launch,
    ])
