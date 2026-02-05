#!/usr/bin/env python3
"""
Camera Simulator Node

Simulates camera images for testing the VIO and satellite matching pipeline
when real hardware is not available.

Publications:
    /vio/camera/image_raw (sensor_msgs/Image) - Simulated camera images
    /vio/camera/camera_info (sensor_msgs/CameraInfo) - Camera intrinsics
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np
import cv2
from pathlib import Path


class CameraSimulator(Node):
    """Simulates camera images for testing."""

    def __init__(self):
        super().__init__('camera_simulator')

        # Parameters
        self.declare_parameter('image_width', 1920)
        self.declare_parameter('image_height', 1080)
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('frame_id', 'camera_optical_frame')
        self.declare_parameter('satellite_image_path', '')
        self.declare_parameter('simulate_motion', True)

        self.width = self.get_parameter('image_width').value
        self.height = self.get_parameter('image_height').value
        self.fps = self.get_parameter('fps').value
        self.frame_id = self.get_parameter('frame_id').value
        self.sat_image_path = self.get_parameter('satellite_image_path').value
        self.simulate_motion = self.get_parameter('simulate_motion').value

        # Publishers
        self.image_pub = self.create_publisher(Image, '/vio/camera/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/vio/camera/camera_info', 10)

        # CV Bridge
        self.bridge = CvBridge()

        # Load satellite image if provided
        self.satellite_image = None
        if self.sat_image_path:
            self.load_satellite_image()
        else:
            # Try default test region
            default_path = Path('/satellite_data/regions/test_region/satellite_synthetic.png')
            if default_path.exists():
                self.sat_image_path = str(default_path)
                self.load_satellite_image()

        # Simulation state
        self.position = np.array([500.0, 500.0])  # Starting position in satellite image
        self.velocity = np.array([2.0, 1.0])  # Pixels per frame
        self.frame_count = 0

        # Camera intrinsics (Arducam IMX477 approximation)
        self.camera_matrix = np.array([
            [1200.0, 0.0, self.width / 2],
            [0.0, 1200.0, self.height / 2],
            [0.0, 0.0, 1.0]
        ])
        self.distortion = np.zeros(5)

        # Timer for publishing
        self.timer = self.create_timer(1.0 / self.fps, self.publish_frame)

        self.get_logger().info(f'Camera simulator started at {self.fps} FPS')
        if self.satellite_image is not None:
            self.get_logger().info(f'Using satellite image: {self.sat_image_path}')

    def load_satellite_image(self):
        """Load satellite image for simulation."""
        try:
            self.satellite_image = cv2.imread(self.sat_image_path)
            if self.satellite_image is not None:
                self.get_logger().info(
                    f'Loaded satellite image: {self.satellite_image.shape}'
                )
            else:
                self.get_logger().warn(f'Failed to load: {self.sat_image_path}')
        except Exception as e:
            self.get_logger().error(f'Error loading satellite image: {e}')

    def generate_synthetic_image(self) -> np.ndarray:
        """Generate a synthetic image with features."""
        image = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        
        # Background
        image[:, :] = [60, 80, 50]
        
        # Add noise
        noise = np.random.randint(-10, 10, image.shape, dtype=np.int16)
        image = np.clip(image.astype(np.int16) + noise, 0, 255).astype(np.uint8)
        
        # Add some moving features (circles)
        np.random.seed(self.frame_count % 100)
        for i in range(20):
            x = int((np.random.rand() * self.width + self.frame_count * 2) % self.width)
            y = int((np.random.rand() * self.height + self.frame_count) % self.height)
            radius = np.random.randint(5, 20)
            color = (
                np.random.randint(100, 200),
                np.random.randint(100, 200),
                np.random.randint(100, 200)
            )
            cv2.circle(image, (x, y), radius, color, -1)
        
        return image

    def extract_view_from_satellite(self) -> np.ndarray:
        """Extract a view from the satellite image based on current position."""
        if self.satellite_image is None:
            return self.generate_synthetic_image()

        sat_h, sat_w = self.satellite_image.shape[:2]
        
        # Update position with motion
        if self.simulate_motion:
            self.position += self.velocity
            
            # Bounce off edges
            if self.position[0] < self.width // 2 or self.position[0] > sat_w - self.width // 2:
                self.velocity[0] *= -1
            if self.position[1] < self.height // 2 or self.position[1] > sat_h - self.height // 2:
                self.velocity[1] *= -1
            
            # Clamp position
            self.position[0] = np.clip(self.position[0], self.width // 2, sat_w - self.width // 2)
            self.position[1] = np.clip(self.position[1], self.height // 2, sat_h - self.height // 2)

        # Extract view
        x1 = int(self.position[0] - self.width // 2)
        y1 = int(self.position[1] - self.height // 2)
        x2 = x1 + self.width
        y2 = y1 + self.height

        # Ensure bounds
        x1 = max(0, x1)
        y1 = max(0, y1)
        x2 = min(sat_w, x2)
        y2 = min(sat_h, y2)

        view = self.satellite_image[y1:y2, x1:x2].copy()
        
        # Resize if needed
        if view.shape[:2] != (self.height, self.width):
            view = cv2.resize(view, (self.width, self.height))

        # Add some simulated aerial effects
        # Slight blur (atmospheric)
        view = cv2.GaussianBlur(view, (3, 3), 0)
        
        # Add noise (sensor)
        noise = np.random.randint(-5, 5, view.shape, dtype=np.int16)
        view = np.clip(view.astype(np.int16) + noise, 0, 255).astype(np.uint8)

        return view

    def publish_frame(self):
        """Publish a simulated camera frame."""
        # Generate image
        if self.satellite_image is not None:
            image = self.extract_view_from_satellite()
        else:
            image = self.generate_synthetic_image()

        # Create timestamp
        stamp = self.get_clock().now().to_msg()

        # Publish image
        img_msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
        img_msg.header.stamp = stamp
        img_msg.header.frame_id = self.frame_id
        self.image_pub.publish(img_msg)

        # Publish camera info
        info_msg = CameraInfo()
        info_msg.header.stamp = stamp
        info_msg.header.frame_id = self.frame_id
        info_msg.width = self.width
        info_msg.height = self.height
        info_msg.distortion_model = 'plumb_bob'
        info_msg.d = self.distortion.tolist()
        info_msg.k = self.camera_matrix.flatten().tolist()
        info_msg.r = np.eye(3).flatten().tolist()
        info_msg.p = np.hstack([self.camera_matrix, np.zeros((3, 1))]).flatten().tolist()
        self.info_pub.publish(info_msg)

        self.frame_count += 1

        if self.frame_count % 100 == 0:
            self.get_logger().info(f'Published {self.frame_count} frames')


def main(args=None):
    rclpy.init(args=args)
    node = CameraSimulator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
