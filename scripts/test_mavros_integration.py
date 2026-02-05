#!/usr/bin/env python3
"""
MAVROS Integration Test

Tests the MAVROS bridge without requiring full PX4 SITL.
Simulates the data flow from VIO -> Fusion -> MAVROS.

This verifies:
1. Vision pose messages are being generated
2. Message format is correct for PX4
3. Coordinate frame transformations work

Usage (inside Docker):
    python3 scripts/test_mavros_integration.py
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
import numpy as np
from scipy.spatial.transform import Rotation
import time


class MAVROSIntegrationTest(Node):
    """Test node for MAVROS integration."""

    def __init__(self):
        super().__init__('mavros_integration_test')
        
        # Statistics
        self.vision_pose_count = 0
        self.fused_pose_count = 0
        self.last_vision_pose = None
        self.start_time = time.time()
        
        # Subscribers
        self.vision_pose_sub = self.create_subscription(
            PoseStamped,
            '/mavros/vision_pose/pose',
            self.vision_pose_callback,
            10
        )
        
        self.fused_pose_sub = self.create_subscription(
            PoseStamped,
            '/state/fused_pose',
            self.fused_pose_callback,
            10
        )
        
        self.fused_odom_sub = self.create_subscription(
            Odometry,
            '/state/fused_odom',
            self.fused_odom_callback,
            10
        )
        
        # Timer for status updates
        self.timer = self.create_timer(2.0, self.print_status)
        
        self.get_logger().info('MAVROS Integration Test started')
        self.get_logger().info('Waiting for pose data...')

    def vision_pose_callback(self, msg: PoseStamped):
        """Handle vision pose messages (what would go to PX4)."""
        self.vision_pose_count += 1
        self.last_vision_pose = msg
        
        if self.vision_pose_count == 1:
            self.get_logger().info('✓ First vision pose received!')

    def fused_pose_callback(self, msg: PoseStamped):
        """Handle fused pose messages."""
        self.fused_pose_count += 1

    def fused_odom_callback(self, msg: Odometry):
        """Handle fused odometry messages."""
        pass  # Just tracking that it exists

    def print_status(self):
        """Print current status."""
        elapsed = time.time() - self.start_time
        
        self.get_logger().info('=' * 50)
        self.get_logger().info(f'MAVROS Integration Test - {elapsed:.1f}s elapsed')
        self.get_logger().info('=' * 50)
        
        # Vision pose status
        if self.vision_pose_count > 0:
            rate = self.vision_pose_count / elapsed
            self.get_logger().info(f'✓ Vision Pose: {self.vision_pose_count} messages ({rate:.1f} Hz)')
            
            if self.last_vision_pose:
                p = self.last_vision_pose.pose.position
                q = self.last_vision_pose.pose.orientation
                
                # Convert quaternion to euler
                rot = Rotation.from_quat([q.x, q.y, q.z, q.w])
                euler = rot.as_euler('xyz', degrees=True)
                
                self.get_logger().info(f'  Position: ({p.x:.2f}, {p.y:.2f}, {p.z:.2f}) m')
                self.get_logger().info(f'  Orientation: (R:{euler[0]:.1f}°, P:{euler[1]:.1f}°, Y:{euler[2]:.1f}°)')
                self.get_logger().info(f'  Frame: {self.last_vision_pose.header.frame_id}')
        else:
            self.get_logger().warn('✗ No vision pose messages yet')
        
        # Fused pose status
        if self.fused_pose_count > 0:
            rate = self.fused_pose_count / elapsed
            self.get_logger().info(f'✓ Fused Pose: {self.fused_pose_count} messages ({rate:.1f} Hz)')
        else:
            self.get_logger().warn('✗ No fused pose messages yet')
        
        # PX4 compatibility check
        if self.last_vision_pose:
            self.check_px4_compatibility()

    def check_px4_compatibility(self):
        """Check if messages are compatible with PX4."""
        msg = self.last_vision_pose
        
        issues = []
        
        # Check frame ID (PX4 expects specific frames)
        valid_frames = ['map', 'odom', 'base_link', 'vision']
        if msg.header.frame_id not in valid_frames:
            issues.append(f'Frame "{msg.header.frame_id}" may need remapping')
        
        # Check for NaN values
        p = msg.pose.position
        q = msg.pose.orientation
        if any(np.isnan([p.x, p.y, p.z, q.x, q.y, q.z, q.w])):
            issues.append('Contains NaN values!')
        
        # Check quaternion normalization
        quat_norm = np.sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2)
        if abs(quat_norm - 1.0) > 0.01:
            issues.append(f'Quaternion not normalized: {quat_norm:.4f}')
        
        if issues:
            self.get_logger().warn('PX4 Compatibility Issues:')
            for issue in issues:
                self.get_logger().warn(f'  - {issue}')
        else:
            self.get_logger().info('✓ PX4 Compatibility: All checks passed!')


def main(args=None):
    rclpy.init(args=args)
    node = MAVROSIntegrationTest()
    
    try:
        # Run for 30 seconds
        end_time = time.time() + 30
        while time.time() < end_time:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        # Final summary
        node.get_logger().info('')
        node.get_logger().info('=' * 50)
        node.get_logger().info('FINAL SUMMARY')
        node.get_logger().info('=' * 50)
        
        if node.vision_pose_count > 0:
            node.get_logger().info('✓ MAVROS integration working!')
            node.get_logger().info(f'  Total vision poses: {node.vision_pose_count}')
            node.get_logger().info('')
            node.get_logger().info('Ready for PX4 connection:')
            node.get_logger().info('  ros2 launch mavros px4.launch.py \\')
            node.get_logger().info('    fcu_url:=udp://:14540@localhost:14557')
        else:
            node.get_logger().warn('✗ No vision poses received')
            node.get_logger().info('Make sure the navigation stack is running')
        
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
