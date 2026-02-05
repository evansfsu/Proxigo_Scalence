#!/usr/bin/env python3
"""
Satellite Matcher Node

Matches aerial camera images against pre-loaded satellite imagery database
to estimate absolute position for GPS-denied navigation.

Subscriptions:
    /vio/camera/image_raw (sensor_msgs/Image) - Camera images
    /sat_match/query_trigger (std_msgs/Header) - Trigger matching
    /mavros/global_position/rel_alt (std_msgs/Float64) - Relative altitude

Publications:
    /sat_match/matched_pose (geometry_msgs/PoseWithCovarianceStamped) - Matched position
    /sat_match/confidence (std_msgs/Float32) - Match confidence score
    /sat_match/debug_image (sensor_msgs/Image) - Debug visualization
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Header, Float32, Float64
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseWithCovarianceStamped

from cv_bridge import CvBridge
import cv2
import numpy as np
import pickle
import json
from pathlib import Path
from dataclasses import dataclass
from typing import Optional, Dict, List, Tuple


@dataclass
class MatchResult:
    """Result of satellite matching."""
    success: bool
    confidence: float
    position_local: Optional[np.ndarray] = None  # [x, y] in local frame
    position_geo: Optional[Tuple[float, float]] = None  # (lon, lat)
    inliers: int = 0
    total_matches: int = 0


class SatelliteMatcherNode(Node):
    """Matches aerial images against satellite database."""

    def __init__(self):
        super().__init__('satellite_matcher_node')

        # Parameters
        self.declare_parameter('satellite_data_path', '/satellite_data')
        self.declare_parameter('region_id', 'test_region')
        self.declare_parameter('initial_lat', 39.678123)
        self.declare_parameter('initial_lon', -75.750456)
        self.declare_parameter('confidence_threshold', 0.3)
        self.declare_parameter('min_matches', 10)
        self.declare_parameter('publish_debug_image', True)

        self.sat_data_path = Path(self.get_parameter('satellite_data_path').value)
        self.region_id = self.get_parameter('region_id').value
        self.initial_lat = self.get_parameter('initial_lat').value
        self.initial_lon = self.get_parameter('initial_lon').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.min_matches = self.get_parameter('min_matches').value
        self.publish_debug = self.get_parameter('publish_debug_image').value

        # CV Bridge
        self.bridge = CvBridge()

        # Feature matcher
        self.orb = cv2.ORB_create(nfeatures=2000)
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

        # Load satellite database
        self.satellite_db = {}
        self.satellite_metadata = {}
        self.load_satellite_data()

        # State
        self.current_altitude = 100.0  # Default altitude
        self.last_image = None
        self.last_image_stamp = None

        # QoS
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscriptions
        self.image_sub = self.create_subscription(
            Image,
            '/vio/camera/image_raw',
            self.image_callback,
            sensor_qos
        )

        self.trigger_sub = self.create_subscription(
            Header,
            '/sat_match/query_trigger',
            self.trigger_callback,
            10
        )

        self.altitude_sub = self.create_subscription(
            Float64,
            '/mavros/global_position/rel_alt',
            self.altitude_callback,
            10
        )

        # Publishers
        self.match_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/sat_match/matched_pose',
            10
        )

        self.confidence_pub = self.create_publisher(
            Float32,
            '/sat_match/confidence',
            10
        )

        self.debug_image_pub = self.create_publisher(
            Image,
            '/sat_match/debug_image',
            10
        )

        self.get_logger().info(f'Satellite matcher initialized for region: {self.region_id}')

    def load_satellite_data(self):
        """Load pre-processed satellite features."""
        region_path = self.sat_data_path / 'regions' / self.region_id

        # Load features
        features_path = region_path / 'features.pkl'
        if features_path.exists():
            with open(features_path, 'rb') as f:
                self.satellite_db = pickle.load(f)
            self.get_logger().info(
                f'Loaded satellite features: {len(self.satellite_db)} altitude scales'
            )
            for alt, data in self.satellite_db.items():
                n_features = len(data.get('keypoints', []))
                self.get_logger().info(f'  Altitude {alt}m: {n_features} features')
        else:
            self.get_logger().warn(f'No satellite features found at {features_path}')

        # Load metadata
        metadata_path = region_path / 'metadata.json'
        if metadata_path.exists():
            with open(metadata_path, 'r') as f:
                self.satellite_metadata = json.load(f)
            self.get_logger().info(f'Loaded metadata for region: {self.satellite_metadata.get("name", "unknown")}')

    def image_callback(self, msg: Image):
        """Store latest image for matching."""
        self.last_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.last_image_stamp = msg.header.stamp

    def altitude_callback(self, msg: Float64):
        """Update current altitude."""
        self.current_altitude = msg.data

    def trigger_callback(self, msg: Header):
        """Triggered to perform satellite matching."""
        if self.last_image is None:
            self.get_logger().warn('No image available for matching')
            return

        # Perform matching
        result = self.match_image(self.last_image)

        # Publish results
        self.publish_results(result, msg.stamp)

    def get_altitude_key(self, altitude: float) -> int:
        """Get closest altitude key from database."""
        available = list(self.satellite_db.keys())
        if not available:
            return 100  # Default
        return min(available, key=lambda x: abs(x - altitude))

    def extract_features(self, image: np.ndarray) -> Tuple[List, Optional[np.ndarray]]:
        """Extract ORB features from aerial image."""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        keypoints, descriptors = self.orb.detectAndCompute(gray, None)
        return keypoints, descriptors

    def match_image(self, image: np.ndarray) -> MatchResult:
        """Match aerial image against satellite database."""
        # Get appropriate altitude scale
        altitude_key = self.get_altitude_key(self.current_altitude)
        
        if altitude_key not in self.satellite_db:
            self.get_logger().warn(f'No data for altitude {altitude_key}m')
            return MatchResult(success=False, confidence=0.0)

        sat_data = self.satellite_db[altitude_key]
        
        if sat_data.get('descriptors') is None or len(sat_data.get('keypoints', [])) == 0:
            self.get_logger().warn(f'No features in satellite data for altitude {altitude_key}m')
            return MatchResult(success=False, confidence=0.0)

        # Extract features from aerial image
        aerial_kps, aerial_descs = self.extract_features(image)
        
        if aerial_descs is None or len(aerial_kps) < self.min_matches:
            self.get_logger().warn(f'Insufficient features in aerial image: {len(aerial_kps) if aerial_kps else 0}')
            return MatchResult(success=False, confidence=0.0)

        # Match features
        try:
            matches = self.matcher.match(aerial_descs, sat_data['descriptors'])
            matches = sorted(matches, key=lambda x: x.distance)
        except cv2.error as e:
            self.get_logger().error(f'Matching error: {e}')
            return MatchResult(success=False, confidence=0.0)

        if len(matches) < self.min_matches:
            return MatchResult(
                success=False,
                confidence=0.0,
                total_matches=len(matches)
            )

        # Get matched points
        src_pts = np.float32([aerial_kps[m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
        
        # Get satellite keypoints (pixel coordinates)
        sat_keypoints = sat_data['keypoints']
        dst_pts = np.float32([
            sat_keypoints[m.trainIdx]['pixel'] if isinstance(sat_keypoints[m.trainIdx], dict) 
            else sat_keypoints[m.trainIdx]
            for m in matches
        ]).reshape(-1, 1, 2)

        # Find homography
        H, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
        
        if H is None:
            return MatchResult(
                success=False,
                confidence=0.0,
                total_matches=len(matches)
            )

        inliers = int(mask.sum())
        confidence = inliers / len(matches)

        if confidence < self.confidence_threshold:
            return MatchResult(
                success=False,
                confidence=confidence,
                inliers=inliers,
                total_matches=len(matches)
            )

        # Transform center of aerial image to satellite coordinates
        h, w = image.shape[:2]
        center = np.array([[[w / 2, h / 2]]], dtype=np.float32)
        transformed = cv2.perspectiveTransform(center, H)[0, 0]

        # Convert to geographic coordinates
        position_geo = self.pixel_to_geo(transformed, sat_data)

        # Convert to local frame (relative to initial position)
        position_local = self.geo_to_local(position_geo)

        return MatchResult(
            success=True,
            confidence=confidence,
            position_local=position_local,
            position_geo=position_geo,
            inliers=inliers,
            total_matches=len(matches)
        )

    def pixel_to_geo(self, pixel: np.ndarray, sat_data: dict) -> Tuple[float, float]:
        """Convert satellite pixel coordinates to geographic coordinates."""
        # Get satellite image dimensions
        img_shape = sat_data.get('original_shape', sat_data.get('image_shape', (2000, 2000, 3)))
        scale_factor = sat_data.get('scale_factor', 1.0)
        
        # Scale pixel back to original satellite image coordinates
        orig_x = pixel[0] / scale_factor
        orig_y = pixel[1] / scale_factor
        
        # Get geographic bounds from metadata
        if self.satellite_metadata and 'center' in self.satellite_metadata:
            center_lat = self.satellite_metadata['center']['latitude']
            center_lon = self.satellite_metadata['center']['longitude']
            
            # Approximate conversion (simplified)
            resolution = self.satellite_metadata.get('imagery', {}).get('resolution_m', 0.5)
            
            # Calculate offset from image center
            img_center_x = img_shape[1] / 2
            img_center_y = img_shape[0] / 2
            
            dx_pixels = orig_x - img_center_x
            dy_pixels = orig_y - img_center_y
            
            # Convert to meters
            dx_m = dx_pixels * resolution
            dy_m = dy_pixels * resolution
            
            # Convert to degrees (approximate)
            # At ~40°N: 1° lon ≈ 85km, 1° lat ≈ 111km
            delta_lon = dx_m / 85000
            delta_lat = -dy_m / 111000  # Negative because y increases downward
            
            return (center_lon + delta_lon, center_lat + delta_lat)
        
        # Fallback: use initial position
        return (self.initial_lon, self.initial_lat)

    def geo_to_local(self, geo: Tuple[float, float]) -> np.ndarray:
        """Convert geographic coordinates to local frame."""
        lon, lat = geo
        
        # Convert to local meters (ENU frame)
        delta_lon = lon - self.initial_lon
        delta_lat = lat - self.initial_lat
        
        # Approximate conversion
        x = delta_lon * 85000  # East
        y = delta_lat * 111000  # North
        
        return np.array([x, y])

    def publish_results(self, result: MatchResult, stamp):
        """Publish matching results."""
        # Publish confidence
        conf_msg = Float32()
        conf_msg.data = result.confidence
        self.confidence_pub.publish(conf_msg)

        if result.success and result.position_local is not None:
            # Publish pose
            pose_msg = PoseWithCovarianceStamped()
            pose_msg.header.stamp = stamp
            pose_msg.header.frame_id = 'map'

            pose_msg.pose.pose.position.x = float(result.position_local[0])
            pose_msg.pose.pose.position.y = float(result.position_local[1])
            pose_msg.pose.pose.position.z = float(self.current_altitude)
            
            # Identity orientation (satellite matching doesn't give heading)
            pose_msg.pose.pose.orientation.w = 1.0

            # Covariance based on confidence
            position_var = 25.0 / max(result.confidence, 0.1)  # Lower confidence = higher variance
            pose_msg.pose.covariance = [
                position_var, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, position_var, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 100.0, 0.0, 0.0, 0.0,  # High Z variance
                0.0, 0.0, 0.0, 999.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 999.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 999.0,
            ]

            self.match_pose_pub.publish(pose_msg)

            self.get_logger().info(
                f'Satellite match: pos=({result.position_local[0]:.1f}, {result.position_local[1]:.1f})m, '
                f'conf={result.confidence:.2f}, inliers={result.inliers}/{result.total_matches}'
            )
        else:
            self.get_logger().debug(
                f'Satellite match failed: conf={result.confidence:.2f}, '
                f'matches={result.total_matches}'
            )

        # Publish debug image
        if self.publish_debug and self.last_image is not None:
            self.publish_debug_visualization(result)

    def publish_debug_visualization(self, result: MatchResult):
        """Publish debug visualization image."""
        if self.last_image is None:
            return

        debug_image = self.last_image.copy()
        
        # Draw info text
        h, w = debug_image.shape[:2]
        
        # Background for text
        cv2.rectangle(debug_image, (10, 10), (400, 120), (0, 0, 0), -1)
        
        # Status
        if result.success:
            status_color = (0, 255, 0)  # Green
            status_text = "MATCHED"
        else:
            status_color = (0, 0, 255)  # Red
            status_text = "NO MATCH"
        
        cv2.putText(debug_image, status_text, (20, 40),
                   cv2.FONT_HERSHEY_SIMPLEX, 1.0, status_color, 2)
        
        cv2.putText(debug_image, f'Confidence: {result.confidence:.2f}', (20, 70),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        
        cv2.putText(debug_image, f'Matches: {result.inliers}/{result.total_matches}', (20, 95),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        
        if result.position_local is not None:
            cv2.putText(debug_image, f'Pos: ({result.position_local[0]:.1f}, {result.position_local[1]:.1f})m', 
                       (20, 115), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        # Publish
        debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
        debug_msg.header.stamp = self.get_clock().now().to_msg()
        self.debug_image_pub.publish(debug_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SatelliteMatcherNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
