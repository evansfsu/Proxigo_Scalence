#!/usr/bin/env python3
"""
MAVROS Bridge Node

Bridges the fused state to PX4 via MAVROS and handles flight control commands.

Subscriptions:
    /state/fused_odom (nav_msgs/Odometry) - Fused global odometry
    /mavros/state (mavros_msgs/State) - FCU state

Publications:
    /mavros/vision_pose/pose (geometry_msgs/PoseStamped) - Vision pose for PX4
    /mavros/setpoint_position/local (geometry_msgs/PoseStamped) - Position setpoints
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TwistStamped

import numpy as np


class MAVROSBridgeNode(Node):
    """Bridges fused state to PX4 via MAVROS."""

    def __init__(self):
        super().__init__('mavros_bridge_node')

        # Parameters
        self.declare_parameter('vision_pose_rate', 30.0)  # Hz
        self.declare_parameter('enable_setpoints', False)
        self.declare_parameter('local_frame', 'map')

        self.vision_rate = self.get_parameter('vision_pose_rate').value
        self.enable_setpoints = self.get_parameter('enable_setpoints').value
        self.local_frame = self.get_parameter('local_frame').value

        # State
        self.current_pose = None
        self.current_velocity = None
        self.fcu_connected = False

        # QoS
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscriptions
        self.odom_sub = self.create_subscription(
            Odometry,
            '/state/fused_odom',
            self.odom_callback,
            sensor_qos
        )

        # Publishers
        self.vision_pose_pub = self.create_publisher(
            PoseStamped, '/mavros/vision_pose/pose', 10
        )

        self.vision_speed_pub = self.create_publisher(
            TwistStamped, '/mavros/vision_speed/speed_twist', 10
        )

        # Timer for publishing vision pose at fixed rate
        self.create_timer(1.0 / self.vision_rate, self.publish_vision_pose)

        self.get_logger().info(f'MAVROS bridge initialized, publishing at {self.vision_rate} Hz')

    def odom_callback(self, msg: Odometry):
        """Store latest odometry for publishing."""
        self.current_pose = msg.pose.pose
        self.current_velocity = msg.twist.twist

    def publish_vision_pose(self):
        """Publish vision pose to MAVROS."""
        if self.current_pose is None:
            return

        # Publish pose
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = self.local_frame
        pose_msg.pose = self.current_pose

        self.vision_pose_pub.publish(pose_msg)

        # Publish velocity
        if self.current_velocity is not None:
            vel_msg = TwistStamped()
            vel_msg.header = pose_msg.header
            vel_msg.twist = self.current_velocity
            self.vision_speed_pub.publish(vel_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MAVROSBridgeNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
