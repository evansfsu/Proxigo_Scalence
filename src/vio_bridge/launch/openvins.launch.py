#!/usr/bin/env python3
"""
OpenVINS Launch File

Launches OpenVINS VIO system with Proxigo configuration.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='/ros2_ws/config/openvins.yaml',
        description='Path to OpenVINS configuration file'
    )
    
    sim_mode_arg = DeclareLaunchArgument(
        'sim_mode',
        default_value='false',
        description='Enable simulation mode with camera/IMU simulators'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Launch RViz for visualization'
    )

    # OpenVINS node
    openvins_node = Node(
        package='ov_msckf',
        executable='run_subscribe_msckf',
        name='ov_msckf',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
        remappings=[
            ('/imu0', '/vio/imu/data'),
            ('/cam0/image_raw', '/vio/camera/image_raw'),
        ]
    )

    # VIO Bridge (processes OpenVINS output)
    vio_bridge_node = Node(
        package='vio_bridge',
        executable='vio_bridge_node',
        name='vio_bridge',
        output='screen',
        parameters=[{
            'vio_topic': '/ov_msckf/odomimu',
            'output_frame': 'odom',
            'body_frame': 'base_link',
            'publish_tf': True,
            'satellite_match_interval': 1.0,
        }]
    )

    # Camera Simulator (only in sim mode)
    camera_sim_node = Node(
        package='vio_bridge',
        executable='camera_simulator',
        name='camera_simulator',
        output='screen',
        condition=IfCondition(LaunchConfiguration('sim_mode')),
        parameters=[{
            'image_width': 1920,
            'image_height': 1080,
            'fps': 30.0,
            'satellite_image_path': '/satellite_data/regions/test_region/satellite_synthetic.png',
        }]
    )

    # IMU Simulator (only in sim mode)
    imu_sim_node = Node(
        package='vio_bridge',
        executable='imu_simulator',
        name='imu_simulator',
        output='screen',
        condition=IfCondition(LaunchConfiguration('sim_mode')),
        parameters=[{
            'rate': 400.0,
            'accel_noise': 0.002,
            'gyro_noise': 0.0002,
        }]
    )

    return LaunchDescription([
        config_file_arg,
        sim_mode_arg,
        use_rviz_arg,
        camera_sim_node,
        imu_sim_node,
        openvins_node,
        vio_bridge_node,
    ])
