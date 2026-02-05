#!/usr/bin/env python3
"""
VIO Bridge Launch File

Launches the VIO bridge node and optionally the camera/IMU simulators.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    # Declare arguments
    sim_mode_arg = DeclareLaunchArgument(
        'sim_mode',
        default_value='false',
        description='Enable simulation mode (camera and IMU simulators)'
    )
    
    vio_topic_arg = DeclareLaunchArgument(
        'vio_topic',
        default_value='/ov_msckf/odomimu',
        description='OpenVINS odometry topic'
    )
    
    sat_match_interval_arg = DeclareLaunchArgument(
        'sat_match_interval',
        default_value='1.0',
        description='Satellite matching trigger interval (seconds)'
    )
    
    satellite_image_path_arg = DeclareLaunchArgument(
        'satellite_image_path',
        default_value='/satellite_data/regions/test_region/satellite_synthetic.png',
        description='Path to satellite image for simulation'
    )

    # VIO Bridge Node
    vio_bridge_node = Node(
        package='vio_bridge',
        executable='vio_bridge_node',
        name='vio_bridge',
        output='screen',
        parameters=[{
            'vio_topic': LaunchConfiguration('vio_topic'),
            'output_frame': 'odom',
            'body_frame': 'base_link',
            'publish_tf': True,
            'satellite_match_interval': LaunchConfiguration('sat_match_interval'),
            'pose_history_size': 100,
        }]
    )

    # Camera Simulator (only in sim mode)
    camera_simulator_node = Node(
        package='vio_bridge',
        executable='camera_simulator',
        name='camera_simulator',
        output='screen',
        condition=IfCondition(LaunchConfiguration('sim_mode')),
        parameters=[{
            'image_width': 1920,
            'image_height': 1080,
            'fps': 30.0,
            'frame_id': 'camera_optical_frame',
            'satellite_image_path': LaunchConfiguration('satellite_image_path'),
            'simulate_motion': True,
        }]
    )

    # IMU Simulator (only in sim mode)
    imu_simulator_node = Node(
        package='vio_bridge',
        executable='imu_simulator',
        name='imu_simulator',
        output='screen',
        condition=IfCondition(LaunchConfiguration('sim_mode')),
        parameters=[{
            'rate': 400.0,
            'frame_id': 'imu_link',
            'accel_noise': 0.002,
            'gyro_noise': 0.0002,
            'simulate_motion': True,
        }]
    )

    return LaunchDescription([
        sim_mode_arg,
        vio_topic_arg,
        sat_match_interval_arg,
        satellite_image_path_arg,
        vio_bridge_node,
        camera_simulator_node,
        imu_simulator_node,
    ])
