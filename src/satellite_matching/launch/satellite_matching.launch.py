#!/usr/bin/env python3
"""
Satellite Matching Launch File

Launches the satellite matching nodes.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare arguments
    satellite_data_path_arg = DeclareLaunchArgument(
        'satellite_data_path',
        default_value='/satellite_data',
        description='Path to satellite data directory'
    )
    
    region_id_arg = DeclareLaunchArgument(
        'region_id',
        default_value='test_region',
        description='Satellite region ID to use'
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
    
    confidence_threshold_arg = DeclareLaunchArgument(
        'confidence_threshold',
        default_value='0.3',
        description='Minimum confidence for valid match'
    )

    # Satellite Matcher Node
    satellite_matcher_node = Node(
        package='satellite_matching',
        executable='satellite_matcher_node',
        name='satellite_matcher',
        output='screen',
        parameters=[{
            'satellite_data_path': LaunchConfiguration('satellite_data_path'),
            'region_id': LaunchConfiguration('region_id'),
            'initial_lat': LaunchConfiguration('initial_lat'),
            'initial_lon': LaunchConfiguration('initial_lon'),
            'confidence_threshold': LaunchConfiguration('confidence_threshold'),
            'min_matches': 10,
            'publish_debug_image': True,
        }]
    )

    # Region Manager Node
    region_manager_node = Node(
        package='satellite_matching',
        executable='region_manager_node',
        name='region_manager',
        output='screen',
        parameters=[{
            'satellite_data_path': LaunchConfiguration('satellite_data_path'),
            'auto_switch_regions': True,
        }]
    )

    return LaunchDescription([
        satellite_data_path_arg,
        region_id_arg,
        initial_lat_arg,
        initial_lon_arg,
        confidence_threshold_arg,
        satellite_matcher_node,
        region_manager_node,
    ])
