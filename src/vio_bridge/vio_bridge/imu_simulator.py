#!/usr/bin/env python3
"""
IMU Simulator Node

Simulates IMU data for testing the VIO pipeline when real hardware is not available.

Publications:
    /vio/imu/data (sensor_msgs/Imu) - Simulated IMU data
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np
from scipy.spatial.transform import Rotation


class IMUSimulator(Node):
    """Simulates IMU data for testing."""

    def __init__(self):
        super().__init__('imu_simulator')

        # Parameters
        self.declare_parameter('rate', 400.0)  # Hz
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('accel_noise', 0.002)  # m/s^2
        self.declare_parameter('gyro_noise', 0.0002)  # rad/s
        self.declare_parameter('accel_bias', [0.0, 0.0, 0.0])
        self.declare_parameter('gyro_bias', [0.0, 0.0, 0.0])
        self.declare_parameter('simulate_motion', True)

        self.rate = self.get_parameter('rate').value
        self.frame_id = self.get_parameter('frame_id').value
        self.accel_noise = self.get_parameter('accel_noise').value
        self.gyro_noise = self.get_parameter('gyro_noise').value
        self.accel_bias = np.array(self.get_parameter('accel_bias').value)
        self.gyro_bias = np.array(self.get_parameter('gyro_bias').value)
        self.enable_motion = self.get_parameter('simulate_motion').value

        # Publisher
        self.imu_pub = self.create_publisher(Imu, '/vio/imu/data', 10)

        # State for simulated motion
        self.time = 0.0
        self.dt = 1.0 / self.rate
        self.orientation = Rotation.identity()
        self.angular_velocity = np.zeros(3)
        self.linear_acceleration = np.array([0.0, 0.0, 9.81])  # Gravity

        # Covariance matrices
        self.orientation_cov = np.eye(3) * 0.01
        self.angular_velocity_cov = np.eye(3) * self.gyro_noise ** 2
        self.linear_acceleration_cov = np.eye(3) * self.accel_noise ** 2

        # Timer
        self.timer = self.create_timer(self.dt, self.publish_imu)
        self.message_count = 0

        self.get_logger().info(f'IMU simulator started at {self.rate} Hz')

    def generate_motion(self) -> tuple:
        """Generate simulated motion (accelerations and angular velocities)."""
        # Simple sinusoidal motion pattern
        freq = 0.1  # Hz
        
        # Angular velocity (gentle rotation)
        wx = 0.01 * np.sin(2 * np.pi * freq * self.time)
        wy = 0.01 * np.sin(2 * np.pi * freq * self.time + np.pi / 4)
        wz = 0.005 * np.sin(2 * np.pi * freq * self.time + np.pi / 2)
        angular_velocity = np.array([wx, wy, wz])
        
        # Linear acceleration (gravity + small motion)
        ax = 0.1 * np.sin(2 * np.pi * freq * 2 * self.time)
        ay = 0.1 * np.cos(2 * np.pi * freq * 2 * self.time)
        az = 9.81 + 0.05 * np.sin(2 * np.pi * freq * 0.5 * self.time)
        linear_acceleration = np.array([ax, ay, az])
        
        return angular_velocity, linear_acceleration

    def publish_imu(self):
        """Publish simulated IMU data."""
        # Get simulated motion
        if self.enable_motion:
            angular_velocity, linear_acceleration = self.generate_motion()
        else:
            angular_velocity = np.zeros(3)
            linear_acceleration = np.array([0.0, 0.0, 9.81])

        # Add noise and bias
        angular_velocity += self.gyro_bias + np.random.randn(3) * self.gyro_noise
        linear_acceleration += self.accel_bias + np.random.randn(3) * self.accel_noise

        # Update orientation (integrate angular velocity)
        delta_rotation = Rotation.from_rotvec(angular_velocity * self.dt)
        self.orientation = self.orientation * delta_rotation

        # Create IMU message
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = self.frame_id

        # Orientation
        q = self.orientation.as_quat()  # [x, y, z, w]
        imu_msg.orientation.x = q[0]
        imu_msg.orientation.y = q[1]
        imu_msg.orientation.z = q[2]
        imu_msg.orientation.w = q[3]
        imu_msg.orientation_covariance = self.orientation_cov.flatten().tolist()

        # Angular velocity
        imu_msg.angular_velocity.x = angular_velocity[0]
        imu_msg.angular_velocity.y = angular_velocity[1]
        imu_msg.angular_velocity.z = angular_velocity[2]
        imu_msg.angular_velocity_covariance = self.angular_velocity_cov.flatten().tolist()

        # Linear acceleration
        imu_msg.linear_acceleration.x = linear_acceleration[0]
        imu_msg.linear_acceleration.y = linear_acceleration[1]
        imu_msg.linear_acceleration.z = linear_acceleration[2]
        imu_msg.linear_acceleration_covariance = self.linear_acceleration_cov.flatten().tolist()

        self.imu_pub.publish(imu_msg)

        self.time += self.dt
        self.message_count += 1

        if self.message_count % 1000 == 0:
            self.get_logger().info(f'Published {self.message_count} IMU messages')


def main(args=None):
    rclpy.init(args=args)
    node = IMUSimulator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
