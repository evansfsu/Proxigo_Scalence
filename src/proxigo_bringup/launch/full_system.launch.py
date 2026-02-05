#!/usr/bin/env python3
"""
Proxigo Scalence - Full System Launch

Launches the complete navigation stack:
- OpenVINS VIO (or simulators in sim mode)
- Satellite Matching
- State Fusion
- MAVROS Bridge

Usage:
    # Simulation mode (no hardware)
    ros2 launch proxigo_bringup full_system.launch.py sim_mode:=true

    # With real hardware
    ros2 launch proxigo_bringup full_system.launch.py

    # With PX4 SITL
    ros2 launch proxigo_bringup full_system.launch.py sim_mode:=true fcu_url:=udp://:14540@localhost:14557
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ============================================================
    # ARGUMENTS
    # ============================================================
    
    sim_mode_arg = DeclareLaunchArgument(
        'sim_mode',
        default_value='true',
        description='Use simulation (camera/IMU simulators) instead of real hardware'
    )
    
    use_openvins_arg = DeclareLaunchArgument(
        'use_openvins',
        default_value='false',
        description='Use OpenVINS for VIO (requires ov_msckf package)'
    )
    
    satellite_data_path_arg = DeclareLaunchArgument(
        'satellite_data_path',
        default_value='/satellite_data',
        description='Path to satellite data directory'
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
    
    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url',
        default_value='',
        description='FCU URL for MAVROS (empty = no MAVROS)'
    )
    
    enable_mavros_arg = DeclareLaunchArgument(
        'enable_mavros',
        default_value='false',
        description='Enable MAVROS connection to PX4'
    )

    # ============================================================
    # VIO SUBSYSTEM (Simulation or OpenVINS)
    # ============================================================
    
    # Camera Simulator (sim mode only)
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
            'frame_id': 'camera_optical_frame',
            'satellite_image_path': PathJoinSubstitution([
                LaunchConfiguration('satellite_data_path'),
                'regions',
                LaunchConfiguration('region_id'),
                'satellite_synthetic.png'
            ]),
            'simulate_motion': True,
        }]
    )
    
    # IMU Simulator (sim mode only)
    imu_sim_node = Node(
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
    
    # VIO Bridge
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
            'pose_history_size': 100,
        }]
    )

    # ============================================================
    # SATELLITE MATCHING SUBSYSTEM
    # ============================================================
    
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
            'confidence_threshold': 0.3,
            'min_matches': 10,
            'publish_debug_image': True,
        }]
    )
    
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

    # ============================================================
    # STATE FUSION SUBSYSTEM
    # ============================================================
    
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
    
    mavros_bridge_node = Node(
        package='state_fusion',
        executable='mavros_bridge_node',
        name='mavros_bridge',
        output='screen',
        parameters=[{
            'vision_pose_rate': 30.0,
            'enable_setpoints': False,
            'local_frame': 'map',
        }]
    )

    # ============================================================
    # MAVROS (Optional)
    # ============================================================
    
    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        name='mavros',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_mavros')),
        parameters=[{
            'fcu_url': LaunchConfiguration('fcu_url'),
            'gcs_url': 'udp://:14550@',
            'system_id': 1,
            'component_id': 1,
        }],
        remappings=[
            ('mavros/vision_pose/pose', '/mavros/vision_pose/pose'),
        ]
    )

    # ============================================================
    # RETURN LAUNCH DESCRIPTION
    # ============================================================
    
    return LaunchDescription([
        # Arguments
        sim_mode_arg,
        use_openvins_arg,
        satellite_data_path_arg,
        region_id_arg,
        initial_lat_arg,
        initial_lon_arg,
        fcu_url_arg,
        enable_mavros_arg,
        
        # VIO Subsystem
        camera_sim_node,
        imu_sim_node,
        vio_bridge_node,
        
        # Satellite Matching
        satellite_matcher_node,
        region_manager_node,
        
        # State Fusion
        fusion_node,
        mavros_bridge_node,
        
        # MAVROS (if enabled)
        mavros_node,
    ])
