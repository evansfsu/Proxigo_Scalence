#!/usr/bin/env python3
"""
VPS Device launch file. Launches the VPS device node (image + altitude -> position).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    reference_path_arg = DeclareLaunchArgument(
        "reference_path",
        default_value="",
        description="Path to Proxigo region directory (with metadata.json)",
    )
    image_topic_arg = DeclareLaunchArgument(
        "image_topic",
        default_value="/vio/camera/image_raw",
        description="Input image topic",
    )
    altitude_topic_arg = DeclareLaunchArgument(
        "altitude_topic",
        default_value="/mavros/global_position/rel_alt",
        description="Relative altitude topic (Float64)",
    )
    match_interval_arg = DeclareLaunchArgument(
        "match_interval_sec",
        default_value="1.0",
        description="Minimum seconds between position estimates",
    )

    vps_device_node = Node(
        package="vps_device",
        executable="vps_device_node",
        name="vps_device_node",
        output="screen",
        parameters=[{
            "reference_path": LaunchConfiguration("reference_path"),
            "image_topic": LaunchConfiguration("image_topic"),
            "altitude_topic": LaunchConfiguration("altitude_topic"),
            "match_interval_sec": LaunchConfiguration("match_interval_sec"),
        }],
    )

    return LaunchDescription([
        reference_path_arg,
        image_topic_arg,
        altitude_topic_arg,
        match_interval_arg,
        vps_device_node,
    ])
