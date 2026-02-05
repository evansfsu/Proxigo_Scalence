#!/usr/bin/env python3
"""
State Fusion Node

Fuses VIO odometry with satellite matching corrections using an Extended Kalman Filter
to produce a globally-corrected pose estimate.

Subscriptions:
    /vio/odom (nav_msgs/Odometry) - VIO odometry (relative motion)
    /sat_match/matched_pose (geometry_msgs/PoseWithCovarianceStamped) - Satellite position

Publications:
    /state/fused_odom (nav_msgs/Odometry) - Fused global odometry
    /state/fused_pose (geometry_msgs/PoseStamped) - Fused pose for visualization
    /mavros/vision_pose/pose (geometry_msgs/PoseStamped) - Pose for PX4 EKF2
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped

from tf2_ros import TransformBroadcaster

import numpy as np
from scipy.spatial.transform import Rotation
from dataclasses import dataclass
from typing import Optional


@dataclass
class FusionState:
    """State vector for fusion EKF."""
    position: np.ndarray  # [x, y, z]
    velocity: np.ndarray  # [vx, vy, vz]
    orientation: np.ndarray  # [qw, qx, qy, qz]
    vio_to_global_translation: np.ndarray  # [tx, ty, tz]
    vio_to_global_yaw: float  # yaw offset


class FusionNode(Node):
    """Fuses VIO with satellite matching for global positioning."""

    def __init__(self):
        super().__init__('fusion_node')

        # Parameters
        self.declare_parameter('vio_position_noise', 0.01)  # m
        self.declare_parameter('vio_velocity_noise', 0.1)  # m/s
        self.declare_parameter('satellite_position_noise', 5.0)  # m
        self.declare_parameter('process_noise_position', 0.1)
        self.declare_parameter('process_noise_velocity', 0.5)
        self.declare_parameter('alignment_learning_rate', 0.1)
        self.declare_parameter('output_frame', 'map')
        self.declare_parameter('body_frame', 'base_link')

        self.vio_pos_noise = self.get_parameter('vio_position_noise').value
        self.vio_vel_noise = self.get_parameter('vio_velocity_noise').value
        self.sat_pos_noise = self.get_parameter('satellite_position_noise').value
        self.proc_noise_pos = self.get_parameter('process_noise_position').value
        self.proc_noise_vel = self.get_parameter('process_noise_velocity').value
        self.align_lr = self.get_parameter('alignment_learning_rate').value
        self.output_frame = self.get_parameter('output_frame').value
        self.body_frame = self.get_parameter('body_frame').value

        # State
        self.state = FusionState(
            position=np.zeros(3),
            velocity=np.zeros(3),
            orientation=np.array([1.0, 0.0, 0.0, 0.0]),
            vio_to_global_translation=np.zeros(3),
            vio_to_global_yaw=0.0
        )

        # Covariance matrix (6x6: position + velocity)
        self.P = np.eye(6) * 100.0  # High initial uncertainty

        # State flags
        self.vio_initialized = False
        self.satellite_initialized = False
        self.last_vio_position = None
        self.last_vio_time = None
        self.message_count = 0

        # QoS
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscriptions
        self.vio_sub = self.create_subscription(
            Odometry,
            '/vio/odom',
            self.vio_callback,
            sensor_qos
        )

        self.sat_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/sat_match/matched_pose',
            self.satellite_callback,
            10
        )

        # Publishers
        self.fused_odom_pub = self.create_publisher(Odometry, '/state/fused_odom', 10)
        self.fused_pose_pub = self.create_publisher(PoseStamped, '/state/fused_pose', 10)
        self.vision_pose_pub = self.create_publisher(
            PoseStamped, '/mavros/vision_pose/pose', 10
        )

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timer for state prediction
        self.create_timer(0.02, self.prediction_step)  # 50Hz
        self.last_prediction_time = self.get_clock().now()

        self.get_logger().info('State fusion node initialized')

    def vio_callback(self, msg: Odometry):
        """Handle VIO odometry updates."""
        self.message_count += 1

        # Extract VIO pose
        vio_pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])

        vio_quat = np.array([
            msg.pose.pose.orientation.w,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z
        ])

        vio_vel = np.array([
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z
        ])

        # Initialize on first message
        if not self.vio_initialized:
            self.last_vio_position = vio_pos
            self.last_vio_time = msg.header.stamp
            self.vio_initialized = True
            self.get_logger().info('VIO initialized')
            return

        # Transform VIO to global frame
        global_pos = self.vio_to_global(vio_pos)
        global_vel = self.rotate_by_yaw(vio_vel, self.state.vio_to_global_yaw)

        # EKF measurement update from VIO
        self.vio_measurement_update(global_pos, global_vel, msg.pose.covariance)

        # Update orientation (direct from VIO, with yaw correction)
        self.state.orientation = self.correct_orientation(vio_quat)

        # Store for next iteration
        self.last_vio_position = vio_pos
        self.last_vio_time = msg.header.stamp

        # Publish fused state
        self.publish_fused_state(msg.header.stamp)

    def satellite_callback(self, msg: PoseWithCovarianceStamped):
        """Handle satellite matching position updates."""
        sat_pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])

        sat_cov = np.array(msg.pose.covariance).reshape(6, 6)[:3, :3]

        # Initialize alignment on first satellite fix
        if not self.satellite_initialized:
            # Set VIO-to-global translation
            if self.vio_initialized and self.last_vio_position is not None:
                self.state.vio_to_global_translation = sat_pos - self.last_vio_position
                self.state.position = sat_pos
                self.P[:3, :3] = sat_cov
                self.satellite_initialized = True
                self.get_logger().info(
                    f'Satellite alignment initialized: offset = {self.state.vio_to_global_translation}'
                )
            return

        # EKF measurement update from satellite
        self.satellite_measurement_update(sat_pos, sat_cov)

        # Refine VIO-to-global alignment
        self.update_alignment(sat_pos)

        self.get_logger().info(
            f'Satellite update: pos=({sat_pos[0]:.1f}, {sat_pos[1]:.1f}, {sat_pos[2]:.1f})'
        )

    def vio_to_global(self, vio_pos: np.ndarray) -> np.ndarray:
        """Transform VIO position to global frame."""
        # Apply yaw rotation
        rotated = self.rotate_by_yaw(vio_pos, self.state.vio_to_global_yaw)
        # Apply translation
        return rotated + self.state.vio_to_global_translation

    def rotate_by_yaw(self, vec: np.ndarray, yaw: float) -> np.ndarray:
        """Rotate a 3D vector by yaw angle."""
        c, s = np.cos(yaw), np.sin(yaw)
        return np.array([
            c * vec[0] - s * vec[1],
            s * vec[0] + c * vec[1],
            vec[2]
        ])

    def correct_orientation(self, vio_quat: np.ndarray) -> np.ndarray:
        """Apply yaw correction to VIO orientation."""
        # Convert to rotation
        vio_rot = Rotation.from_quat([vio_quat[1], vio_quat[2], vio_quat[3], vio_quat[0]])
        
        # Apply yaw correction
        yaw_rot = Rotation.from_euler('z', self.state.vio_to_global_yaw)
        corrected = yaw_rot * vio_rot
        
        # Convert back to quaternion [w, x, y, z]
        q = corrected.as_quat()
        return np.array([q[3], q[0], q[1], q[2]])

    def prediction_step(self):
        """EKF prediction step."""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_prediction_time).nanoseconds / 1e9
        self.last_prediction_time = current_time

        if dt > 0.1 or dt <= 0:  # Skip if too long or invalid
            return

        # State prediction: position += velocity * dt
        self.state.position += self.state.velocity * dt

        # State transition matrix
        F = np.eye(6)
        F[0, 3] = dt
        F[1, 4] = dt
        F[2, 5] = dt

        # Process noise
        Q = np.zeros((6, 6))
        Q[:3, :3] = np.eye(3) * self.proc_noise_pos * dt
        Q[3:, 3:] = np.eye(3) * self.proc_noise_vel * dt

        # Covariance prediction
        self.P = F @ self.P @ F.T + Q

    def vio_measurement_update(self, pos: np.ndarray, vel: np.ndarray, cov):
        """EKF measurement update from VIO."""
        # Measurement vector
        z = np.concatenate([pos, vel])

        # Measurement model (identity)
        H = np.eye(6)

        # Measurement noise
        R = np.eye(6)
        R[:3, :3] *= self.vio_pos_noise ** 2
        R[3:, 3:] *= self.vio_vel_noise ** 2

        # Innovation
        x = np.concatenate([self.state.position, self.state.velocity])
        y = z - H @ x

        # Innovation covariance
        S = H @ self.P @ H.T + R

        # Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)

        # State update
        x_new = x + K @ y
        self.state.position = x_new[:3]
        self.state.velocity = x_new[3:]

        # Covariance update
        self.P = (np.eye(6) - K @ H) @ self.P

    def satellite_measurement_update(self, sat_pos: np.ndarray, sat_cov: np.ndarray):
        """EKF measurement update from satellite position."""
        # Measurement model (position only)
        H = np.zeros((3, 6))
        H[:3, :3] = np.eye(3)

        # Measurement noise
        R = sat_cov

        # Innovation
        x = np.concatenate([self.state.position, self.state.velocity])
        y = sat_pos - H @ x

        # Innovation covariance
        S = H @ self.P @ H.T + R

        # Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)

        # State update
        x_new = x + K @ y
        self.state.position = x_new[:3]
        self.state.velocity = x_new[3:]

        # Covariance update
        self.P = (np.eye(6) - K @ H) @ self.P

    def update_alignment(self, sat_pos: np.ndarray):
        """Update VIO-to-global alignment based on satellite fix."""
        # Compute correction needed
        error = sat_pos - self.state.position

        # Gradually update translation
        self.state.vio_to_global_translation += self.align_lr * error

    def publish_fused_state(self, stamp):
        """Publish fused state."""
        # Odometry message
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.output_frame
        odom.child_frame_id = self.body_frame

        odom.pose.pose.position.x = float(self.state.position[0])
        odom.pose.pose.position.y = float(self.state.position[1])
        odom.pose.pose.position.z = float(self.state.position[2])

        odom.pose.pose.orientation.w = float(self.state.orientation[0])
        odom.pose.pose.orientation.x = float(self.state.orientation[1])
        odom.pose.pose.orientation.y = float(self.state.orientation[2])
        odom.pose.pose.orientation.z = float(self.state.orientation[3])

        odom.twist.twist.linear.x = float(self.state.velocity[0])
        odom.twist.twist.linear.y = float(self.state.velocity[1])
        odom.twist.twist.linear.z = float(self.state.velocity[2])

        # Covariance
        odom.pose.covariance = [0.0] * 36
        for i in range(3):
            for j in range(3):
                odom.pose.covariance[i * 6 + j] = float(self.P[i, j])

        self.fused_odom_pub.publish(odom)

        # Pose message
        pose = PoseStamped()
        pose.header = odom.header
        pose.pose = odom.pose.pose
        self.fused_pose_pub.publish(pose)

        # Vision pose for MAVROS/PX4
        vision_pose = PoseStamped()
        vision_pose.header.stamp = stamp
        vision_pose.header.frame_id = self.output_frame
        vision_pose.pose = odom.pose.pose
        self.vision_pose_pub.publish(vision_pose)

        # Publish TF
        self.publish_transform(odom)

        # Log periodically
        if self.message_count % 50 == 0:
            self.get_logger().info(
                f'Fused: pos=({self.state.position[0]:.2f}, {self.state.position[1]:.2f}, {self.state.position[2]:.2f}), '
                f'vel=({self.state.velocity[0]:.2f}, {self.state.velocity[1]:.2f}, {self.state.velocity[2]:.2f})'
            )

    def publish_transform(self, odom: Odometry):
        """Publish TF transform."""
        t = TransformStamped()
        t.header = odom.header
        t.child_frame_id = odom.child_frame_id

        t.transform.translation.x = odom.pose.pose.position.x
        t.transform.translation.y = odom.pose.pose.position.y
        t.transform.translation.z = odom.pose.pose.position.z
        t.transform.rotation = odom.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = FusionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
