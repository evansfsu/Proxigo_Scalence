#!/usr/bin/env python3
"""
VIO Bridge Node

This node serves as the interface between OpenVINS VIO output and the
downstream navigation components (satellite matching, state fusion, MAVROS).

Subscriptions:
    /ov_msckf/odomimu (nav_msgs/Odometry) - OpenVINS odometry output
    /ov_msckf/poseimu (geometry_msgs/PoseWithCovarianceStamped) - OpenVINS pose

Publications:
    /vio/odom (nav_msgs/Odometry) - Processed VIO odometry
    /vio/pose (geometry_msgs/PoseStamped) - VIO pose for visualization
    /vio/pose_with_cov (geometry_msgs/PoseWithCovarianceStamped) - VIO pose with covariance
    /sat_match/query_trigger (std_msgs/Header) - Trigger satellite matching
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped
from sensor_msgs.msg import Image

from tf2_ros import TransformBroadcaster

import numpy as np
from collections import deque


class VIOBridgeNode(Node):
    """Bridge between OpenVINS and downstream navigation components."""

    def __init__(self):
        super().__init__('vio_bridge_node')

        # Parameters
        self.declare_parameter('vio_topic', '/ov_msckf/odomimu')
        self.declare_parameter('output_frame', 'odom')
        self.declare_parameter('body_frame', 'base_link')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('satellite_match_interval', 1.0)  # seconds
        self.declare_parameter('pose_history_size', 100)
        self.declare_parameter('sim_mode', True)  # Generate simulated VIO when no real data

        self.vio_topic = self.get_parameter('vio_topic').value
        self.output_frame = self.get_parameter('output_frame').value
        self.body_frame = self.get_parameter('body_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.sat_match_interval = self.get_parameter('satellite_match_interval').value
        self.pose_history_size = self.get_parameter('pose_history_size').value
        self.sim_mode = self.get_parameter('sim_mode').value

        # QoS profile for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscriptions
        self.vio_sub = self.create_subscription(
            Odometry,
            self.vio_topic,
            self.vio_callback,
            sensor_qos
        )

        # Publications
        self.odom_pub = self.create_publisher(Odometry, '/vio/odom', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/vio/pose', 10)
        self.pose_cov_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/vio/pose_with_cov', 10
        )
        self.sat_trigger_pub = self.create_publisher(Header, '/sat_match/query_trigger', 10)

        # TF broadcaster
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)

        # State
        self.pose_history = deque(maxlen=self.pose_history_size)
        self.last_sat_match_time = self.get_clock().now()
        self.vio_initialized = False
        self.message_count = 0

        # Statistics
        self.position_drift = np.zeros(3)
        self.last_position = None

        # Simulation mode: generate VIO when no real data
        self.sim_position = np.array([0.0, 0.0, 50.0])  # Start at 50m altitude
        self.sim_velocity = np.array([5.0, 0.0, 0.0])  # 5 m/s forward
        self.sim_yaw = 0.0
        self.sim_time = 0.0
        
        if self.sim_mode:
            # Timer to generate simulated VIO at 50Hz
            self.sim_timer = self.create_timer(0.02, self.publish_simulated_vio)
            self.get_logger().info('VIO Bridge in SIMULATION mode - generating VIO data')
        
        self.get_logger().info(f'VIO Bridge initialized, subscribing to {self.vio_topic}')

    def vio_callback(self, msg: Odometry):
        """Process VIO odometry and republish."""
        self.message_count += 1

        if not self.vio_initialized:
            self.vio_initialized = True
            self.get_logger().info('VIO Bridge: First message received, VIO initialized')

        # Store in history
        self.pose_history.append({
            'timestamp': msg.header.stamp,
            'position': np.array([
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z
            ]),
            'orientation': np.array([
                msg.pose.pose.orientation.w,
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z
            ]),
            'velocity': np.array([
                msg.twist.twist.linear.x,
                msg.twist.twist.linear.y,
                msg.twist.twist.linear.z
            ])
        })

        # Update frame IDs
        output_odom = Odometry()
        output_odom.header = msg.header
        output_odom.header.frame_id = self.output_frame
        output_odom.child_frame_id = self.body_frame
        output_odom.pose = msg.pose
        output_odom.twist = msg.twist

        # Publish odometry
        self.odom_pub.publish(output_odom)

        # Publish pose
        pose_msg = PoseStamped()
        pose_msg.header = output_odom.header
        pose_msg.pose = msg.pose.pose
        self.pose_pub.publish(pose_msg)

        # Publish pose with covariance
        pose_cov_msg = PoseWithCovarianceStamped()
        pose_cov_msg.header = output_odom.header
        pose_cov_msg.pose = msg.pose
        self.pose_cov_pub.publish(pose_cov_msg)

        # Publish TF
        if self.publish_tf:
            self.publish_transform(output_odom)

        # Check if we should trigger satellite matching
        self.check_satellite_match_trigger(msg.header)

        # Log periodically
        if self.message_count % 100 == 0:
            pos = msg.pose.pose.position
            self.get_logger().info(
                f'VIO: pos=({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f}), '
                f'msgs={self.message_count}'
            )

    def publish_transform(self, odom: Odometry):
        """Publish TF transform from odom frame to body frame."""
        t = TransformStamped()
        t.header = odom.header
        t.child_frame_id = odom.child_frame_id

        t.transform.translation.x = odom.pose.pose.position.x
        t.transform.translation.y = odom.pose.pose.position.y
        t.transform.translation.z = odom.pose.pose.position.z
        t.transform.rotation = odom.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)

    def check_satellite_match_trigger(self, header: Header):
        """Check if it's time to trigger satellite matching."""
        current_time = self.get_clock().now()
        elapsed = (current_time - self.last_sat_match_time).nanoseconds / 1e9

        if elapsed >= self.sat_match_interval:
            # Trigger satellite matching
            trigger_msg = Header()
            trigger_msg.stamp = header.stamp
            trigger_msg.frame_id = self.output_frame
            self.sat_trigger_pub.publish(trigger_msg)
            self.last_sat_match_time = current_time

    def publish_simulated_vio(self):
        """Publish simulated VIO odometry for testing without OpenVINS."""
        dt = 0.02  # 50Hz
        self.sim_time += dt
        
        # Simple circular motion pattern
        radius = 100.0  # meters
        angular_speed = 0.05  # rad/s
        
        # Update position (circular path)
        self.sim_yaw += angular_speed * dt
        self.sim_position[0] = radius * np.cos(self.sim_yaw)
        self.sim_position[1] = radius * np.sin(self.sim_yaw)
        self.sim_position[2] = 50.0 + 5.0 * np.sin(self.sim_time * 0.2)  # Altitude variation
        
        # Velocity (tangent to circle)
        speed = radius * angular_speed
        self.sim_velocity[0] = -speed * np.sin(self.sim_yaw)
        self.sim_velocity[1] = speed * np.cos(self.sim_yaw)
        self.sim_velocity[2] = 5.0 * 0.2 * np.cos(self.sim_time * 0.2)
        
        # Create odometry message
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.output_frame
        odom.child_frame_id = self.body_frame
        
        odom.pose.pose.position.x = float(self.sim_position[0])
        odom.pose.pose.position.y = float(self.sim_position[1])
        odom.pose.pose.position.z = float(self.sim_position[2])
        
        # Orientation (heading in direction of travel)
        odom.pose.pose.orientation.w = float(np.cos(self.sim_yaw / 2))
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = float(np.sin(self.sim_yaw / 2))
        
        odom.twist.twist.linear.x = float(self.sim_velocity[0])
        odom.twist.twist.linear.y = float(self.sim_velocity[1])
        odom.twist.twist.linear.z = float(self.sim_velocity[2])
        
        # Small covariance (simulated VIO is "perfect")
        odom.pose.covariance[0] = 0.01  # x
        odom.pose.covariance[7] = 0.01  # y
        odom.pose.covariance[14] = 0.01  # z
        
        # Publish
        self.odom_pub.publish(odom)
        
        # Also publish pose
        pose_msg = PoseStamped()
        pose_msg.header = odom.header
        pose_msg.pose = odom.pose.pose
        self.pose_pub.publish(pose_msg)
        
        # Trigger satellite matching periodically
        self.check_satellite_match_trigger(odom.header)
        
        # Log occasionally
        if int(self.sim_time * 50) % 100 == 0:
            self.get_logger().info(
                f'Sim VIO: pos=({self.sim_position[0]:.1f}, {self.sim_position[1]:.1f}, {self.sim_position[2]:.1f})'
            )

    def get_pose_at_time(self, target_time):
        """Get interpolated pose at a specific time."""
        if len(self.pose_history) < 2:
            return None

        # Find bracketing poses
        for i in range(len(self.pose_history) - 1):
            t1 = self.pose_history[i]['timestamp']
            t2 = self.pose_history[i + 1]['timestamp']

            t1_sec = t1.sec + t1.nanosec / 1e9
            t2_sec = t2.sec + t2.nanosec / 1e9
            target_sec = target_time.sec + target_time.nanosec / 1e9

            if t1_sec <= target_sec <= t2_sec:
                # Linear interpolation
                alpha = (target_sec - t1_sec) / (t2_sec - t1_sec)
                
                pos = (1 - alpha) * self.pose_history[i]['position'] + \
                      alpha * self.pose_history[i + 1]['position']
                
                # SLERP for orientation would be better, but linear is ok for small intervals
                orient = (1 - alpha) * self.pose_history[i]['orientation'] + \
                         alpha * self.pose_history[i + 1]['orientation']
                orient = orient / np.linalg.norm(orient)  # Normalize

                return {'position': pos, 'orientation': orient}

        return None


def main(args=None):
    rclpy.init(args=args)
    node = VIOBridgeNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
