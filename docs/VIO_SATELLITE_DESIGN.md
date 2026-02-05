# Visual Inertial Odometry with Satellite Region Referencing

## Overview

This document describes the Visual Inertial Odometry (VIO) system enhanced with satellite imagery region referencing for absolute position correction in GPS-denied environments. The system combines:

1. **OpenVINS** - Real-time visual-inertial state estimation
2. **Satellite Matching** - Aerial-to-satellite image correlation for absolute positioning
3. **Fusion Layer** - Combining relative VIO with absolute satellite corrections

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                        VIO + SATELLITE MATCHING PIPELINE                    │
│                                                                              │
│  ┌──────────────┐    ┌──────────────────────────────────────────────────┐  │
│  │   Arducam    │    │              PERCEPTION PIPELINE                  │  │
│  │   IMX477     │───►│  ┌────────────┐      ┌─────────────────────┐     │  │
│  │   Camera     │    │  │  Feature   │      │   OpenVINS Core     │     │  │
│  └──────────────┘    │  │ Extraction │─────►│                     │     │  │
│                      │  │  (GPU)     │      │  - MSCKF VIO        │     │  │
│  ┌──────────────┐    │  └────────────┘      │  - IMU Preintegration│    │  │
│  │  External    │    │                      │  - Sliding Window   │     │  │
│  │    IMU       │───►│──────────────────────►  - Covariance Est.  │     │  │
│  │ (200-400Hz)  │    │                      │                     │     │  │
│  └──────────────┘    │                      └──────────┬──────────┘     │  │
│                      │                                 │                 │  │
│                      │              ┌──────────────────▼───────────────┐ │  │
│                      │              │      Relative Pose Output        │ │  │
│                      │              │   (Position, Velocity, Bias)     │ │  │
│                      │              └──────────────────┬───────────────┘ │  │
│                      └─────────────────────────────────┼─────────────────┘  │
│                                                        │                     │
│  ┌─────────────────────────────────────────────────────┼─────────────────┐  │
│  │                    SATELLITE MATCHING PIPELINE       │                 │  │
│  │                                                      │                 │  │
│  │  ┌──────────────────┐    ┌────────────────────┐    │                 │  │
│  │  │ Satellite Image  │    │  Feature Matching  │    │                 │  │
│  │  │ Database         │───►│  Engine            │◄───┼─ Current Image  │  │
│  │  │ (Pre-loaded)     │    │  (NetVLAD/SuperGlue)│   │                 │  │
│  │  └──────────────────┘    └─────────┬──────────┘    │                 │  │
│  │                                    │                │                 │  │
│  │  ┌──────────────────┐    ┌─────────▼──────────┐    │                 │  │
│  │  │ User Input:      │    │  Geo-Referencing   │    │                 │  │
│  │  │ Initial Coords   │───►│  Transform         │    │                 │  │
│  │  │ (Lat/Lon/Alt)    │    │  (Pixel → WGS84)   │    │                 │  │
│  │  └──────────────────┘    └─────────┬──────────┘    │                 │  │
│  │                                    │                │                 │  │
│  │                         ┌──────────▼───────────┐   │                 │  │
│  │                         │  Absolute Position   │   │                 │  │
│  │                         │  + Confidence Score  │   │                 │  │
│  │                         └──────────┬───────────┘   │                 │  │
│  └─────────────────────────────────────┼──────────────┼─────────────────┘  │
│                                        │              │                     │
│  ┌─────────────────────────────────────▼──────────────▼─────────────────┐  │
│  │                         FUSION LAYER                                  │  │
│  │                                                                       │  │
│  │  ┌───────────────────────────────────────────────────────────────┐   │  │
│  │  │                    Extended Kalman Filter                      │   │  │
│  │  │                                                                │   │  │
│  │  │  State: [Position, Velocity, Orientation, IMU Biases,          │   │  │
│  │  │          VIO-to-Global Transform]                              │   │  │
│  │  │                                                                │   │  │
│  │  │  Prediction: VIO relative motion                               │   │  │
│  │  │  Update: Satellite absolute position (when available)          │   │  │
│  │  │                                                                │   │  │
│  │  └───────────────────────────────────────────────────────────────┘   │  │
│  │                                    │                                  │  │
│  │                         ┌──────────▼───────────┐                     │  │
│  │                         │   Fused Global Pose  │                     │  │
│  │                         │   → PX4 EKF2 Input   │                     │  │
│  │                         └──────────────────────┘                     │  │
│  └───────────────────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## VIO Core: OpenVINS Integration

### Why OpenVINS?

| Feature | Benefit |
|---------|---------|
| MSCKF Architecture | Computationally efficient, bounded complexity |
| Modular Design | Easy to extend with satellite matching |
| ROS2 Support | Native integration with our stack |
| Calibration | Online camera-IMU calibration |
| Well-documented | Extensive academic documentation |

### OpenVINS Configuration for Arducam IMX477

```yaml
# config/vio/openvins_config.yaml

# Camera Configuration
cam0:
  camera_model: "pinhole"
  distortion_model: "radtan"
  resolution: [1920, 1080]  # 1080p @ 60fps for VIO
  intrinsics: [1200.0, 1200.0, 960.0, 540.0]  # fx, fy, cx, cy
  distortion_coeffs: [0.0, 0.0, 0.0, 0.0]  # k1, k2, p1, p2
  
# IMU Configuration  
imu0:
  accelerometer_noise_density: 0.002
  accelerometer_random_walk: 0.0002
  gyroscope_noise_density: 0.00016
  gyroscope_random_walk: 0.000022
  update_rate: 400  # Hz

# Camera-IMU Extrinsics (to be calibrated)
T_imu_cam0:
  - [1.0, 0.0, 0.0, 0.05]
  - [0.0, 1.0, 0.0, 0.0]
  - [0.0, 0.0, 1.0, 0.0]
  - [0.0, 0.0, 0.0, 1.0]

# Time offset (camera to IMU, seconds)
timeshift_cam_imu: 0.0

# Feature tracking
num_pts: 200
fast_threshold: 20
grid_x: 5
grid_y: 5
min_px_dist: 15

# State estimation
use_fej: true
use_imuavg: true
use_rk4int: true
calib_cam_extrinsics: false  # Set true for online calibration
calib_cam_intrinsics: false
calib_cam_timeoffset: true
max_clones: 11
max_slam: 50
max_aruco: 1024
```

### Custom VIO Output Interface

```python
# src/vio_core/vio_bridge.py
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Imu, Image
import numpy as np

class VIOBridge(Node):
    """Bridge between OpenVINS and downstream consumers."""
    
    def __init__(self):
        super().__init__('vio_bridge')
        
        # OpenVINS output subscription
        self.vio_sub = self.create_subscription(
            Odometry,
            '/ov_msckf/odomimu',
            self.vio_callback,
            10
        )
        
        # Publishers
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/vio/pose_with_covariance',
            10
        )
        
        # For satellite matching
        self.query_image_pub = self.create_publisher(
            Image,
            '/sat_match/query_image',
            1
        )
        
        # State
        self.vio_pose_history = []
        self.last_sat_match_time = None
        self.sat_match_interval = 2.0  # seconds
        
    def vio_callback(self, msg: Odometry):
        # Convert to PoseWithCovarianceStamped
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header = msg.header
        pose_msg.pose.pose = msg.pose.pose
        pose_msg.pose.covariance = msg.pose.covariance
        
        self.pose_pub.publish(pose_msg)
        
        # Store for trajectory
        self.vio_pose_history.append({
            'timestamp': msg.header.stamp,
            'pose': msg.pose.pose,
            'covariance': np.array(msg.pose.covariance).reshape(6, 6)
        })
        
        # Limit history size
        if len(self.vio_pose_history) > 1000:
            self.vio_pose_history.pop(0)
```

---

## Satellite Region Referencing System

### Concept Overview

The satellite matching system correlates downward-looking camera images with pre-downloaded satellite/aerial imagery to determine absolute position. This serves as a "GPS-like" correction to bound VIO drift.

```
                    USER WORKFLOW
                    
    1. Pre-Flight Setup
       ┌────────────────────────────────────────┐
       │  • Download satellite imagery for       │
       │    mission area (Google Earth, etc.)   │
       │  • Define region bounds (GeoJSON)      │
       │  • Input initial takeoff coordinates   │
       │  • Load to /satellite_data volume      │
       └────────────────────────────────────────┘
                         │
                         ▼
    2. System Initialization
       ┌────────────────────────────────────────┐
       │  • Load satellite imagery database     │
       │  • Pre-compute feature descriptors     │
       │  • Initialize coordinate transforms    │
       │  • Set VIO origin at initial coords    │
       └────────────────────────────────────────┘
                         │
                         ▼
    3. Runtime Operation
       ┌────────────────────────────────────────┐
       │  • Periodic aerial image capture       │
       │  • Match against satellite database    │
       │  • Compute absolute position           │
       │  • Fuse with VIO for corrected pose    │
       └────────────────────────────────────────┘
```

### Satellite Data Preparation

#### Region Definition: `metadata.json`

```json
{
  "region_id": "mission_area_001",
  "name": "Test Flight Zone Alpha",
  "bounds": {
    "type": "Polygon",
    "coordinates": [[
      [-76.1234, 39.5678],
      [-76.1234, 39.5789],
      [-76.1123, 39.5789],
      [-76.1123, 39.5678],
      [-76.1234, 39.5678]
    ]]
  },
  "crs": "EPSG:4326",
  "imagery": {
    "source": "Google Earth",
    "capture_date": "2024-06-15",
    "resolution_m": 0.3,
    "file": "satellite.tif"
  },
  "elevation": {
    "source": "SRTM",
    "resolution_m": 30,
    "file": "elevation.tif"
  }
}
```

#### Satellite Image Processing Pipeline

```python
# src/satellite_matching/preprocessing/prepare_region.py
import rasterio
import numpy as np
import cv2
from pathlib import Path
import json

class RegionPreprocessor:
    """Prepare satellite imagery for matching."""
    
    def __init__(self, region_path: Path):
        self.region_path = region_path
        self.metadata = self._load_metadata()
        
    def _load_metadata(self) -> dict:
        with open(self.region_path / 'metadata.json') as f:
            return json.load(f)
    
    def process(self):
        """Full preprocessing pipeline."""
        # Load satellite image
        sat_image = self._load_geotiff()
        
        # Extract features at multiple scales
        features = self._extract_multiscale_features(sat_image)
        
        # Build spatial index for fast lookup
        spatial_index = self._build_spatial_index(features)
        
        # Save processed data
        self._save_processed(features, spatial_index)
        
    def _load_geotiff(self) -> tuple:
        """Load GeoTIFF with georeferencing."""
        tif_path = self.region_path / self.metadata['imagery']['file']
        
        with rasterio.open(tif_path) as src:
            image = src.read()  # (bands, height, width)
            transform = src.transform
            crs = src.crs
            
        # Convert to RGB if needed
        if image.shape[0] >= 3:
            image = np.moveaxis(image[:3], 0, -1)  # (H, W, 3)
        
        return image, transform, crs
    
    def _extract_multiscale_features(self, sat_data: tuple) -> dict:
        """Extract features at multiple altitude scales."""
        image, transform, crs = sat_data
        
        # Altitude ranges (meters AGL)
        altitude_scales = [50, 100, 200, 400]
        features = {}
        
        for alt in altitude_scales:
            # Compute expected view size at altitude
            fov_h = 60  # degrees (approximate for 6mm lens)
            view_width_m = 2 * alt * np.tan(np.radians(fov_h / 2))
            
            # Downsample to match expected resolution
            scale_factor = view_width_m / (image.shape[1] * self.metadata['imagery']['resolution_m'])
            
            if scale_factor < 1:
                scaled = cv2.resize(image, None, fx=scale_factor, fy=scale_factor)
            else:
                scaled = image
            
            # Extract SuperPoint features
            features[alt] = self._extract_superpoint(scaled, transform, alt)
        
        return features
    
    def _extract_superpoint(self, image: np.ndarray, transform, altitude: int) -> dict:
        """Extract SuperPoint features for matching."""
        # Using OpenCV ORB as fallback (SuperPoint requires PyTorch)
        orb = cv2.ORB_create(nfeatures=5000)
        gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        
        keypoints, descriptors = orb.detectAndCompute(gray, None)
        
        # Convert keypoints to geo-coordinates
        geo_keypoints = []
        for kp in keypoints:
            # Pixel to geo transform
            lon, lat = rasterio.transform.xy(transform, kp.pt[1], kp.pt[0])
            geo_keypoints.append({
                'pixel': (kp.pt[0], kp.pt[1]),
                'geo': (lon, lat),
                'response': kp.response,
                'size': kp.size
            })
        
        return {
            'keypoints': geo_keypoints,
            'descriptors': descriptors,
            'altitude': altitude,
            'image_shape': image.shape
        }
```

### Real-Time Matching Node

```python
# src/satellite_matching/nodes/satellite_matcher_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import cv2
import numpy as np
from pyproj import Transformer
import torch

class SatelliteMatcherNode(Node):
    """Match aerial images against satellite database."""
    
    def __init__(self):
        super().__init__('satellite_matcher')
        
        # Parameters
        self.declare_parameter('satellite_data_path', '/satellite_data')
        self.declare_parameter('initial_lat', 0.0)
        self.declare_parameter('initial_lon', 0.0)
        self.declare_parameter('initial_alt', 0.0)
        self.declare_parameter('match_confidence_threshold', 0.7)
        
        self.sat_data_path = self.get_parameter('satellite_data_path').value
        self.initial_coords = (
            self.get_parameter('initial_lat').value,
            self.get_parameter('initial_lon').value,
            self.get_parameter('initial_alt').value
        )
        self.confidence_threshold = self.get_parameter('match_confidence_threshold').value
        
        # Initialize
        self.bridge = CvBridge()
        self.transformer = Transformer.from_crs("EPSG:4326", "EPSG:32618")  # UTM zone
        self.satellite_db = self._load_satellite_database()
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        
        # Subscriptions
        self.image_sub = self.create_subscription(
            Image,
            '/vio/camera/image_raw',
            self.image_callback,
            1  # Only keep latest
        )
        
        self.altitude_sub = self.create_subscription(
            Float32,
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
        
        # State
        self.current_altitude = 100.0  # Default
        self.last_match_time = self.get_clock().now()
        self.match_interval = rclpy.duration.Duration(seconds=1.0)
        
        self.get_logger().info('Satellite matcher initialized')
        
    def _load_satellite_database(self) -> dict:
        """Load pre-processed satellite features."""
        import pickle
        from pathlib import Path
        
        db_path = Path(self.sat_data_path) / 'features.pkl'
        if db_path.exists():
            with open(db_path, 'rb') as f:
                return pickle.load(f)
        else:
            self.get_logger().warn(f'No satellite database at {db_path}')
            return {}
    
    def altitude_callback(self, msg: Float32):
        self.current_altitude = msg.data
        
    def image_callback(self, msg: Image):
        # Rate limit matching
        now = self.get_clock().now()
        if (now - self.last_match_time) < self.match_interval:
            return
            
        self.last_match_time = now
        
        # Convert image
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        
        # Find best matching altitude scale
        altitude_key = self._get_altitude_key(self.current_altitude)
        
        if altitude_key not in self.satellite_db:
            self.get_logger().warn(f'No satellite data for altitude {altitude_key}m')
            return
        
        # Extract features from aerial image
        aerial_features = self._extract_features(cv_image)
        
        # Match against satellite database
        match_result = self._match_features(aerial_features, self.satellite_db[altitude_key])
        
        if match_result['confidence'] > self.confidence_threshold:
            # Publish matched pose
            self._publish_matched_pose(match_result, msg.header.stamp)
            
    def _get_altitude_key(self, altitude: float) -> int:
        """Get closest altitude key from database."""
        available = [50, 100, 200, 400]
        return min(available, key=lambda x: abs(x - altitude))
    
    def _extract_features(self, image: np.ndarray) -> dict:
        """Extract ORB features from aerial image."""
        orb = cv2.ORB_create(nfeatures=2000)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        keypoints, descriptors = orb.detectAndCompute(gray, None)
        
        return {
            'keypoints': keypoints,
            'descriptors': descriptors,
            'image_shape': image.shape
        }
    
    def _match_features(self, aerial: dict, satellite: dict) -> dict:
        """Match aerial features against satellite database."""
        if aerial['descriptors'] is None or satellite['descriptors'] is None:
            return {'confidence': 0.0, 'position': None}
        
        # Feature matching
        matches = self.matcher.match(aerial['descriptors'], satellite['descriptors'])
        matches = sorted(matches, key=lambda x: x.distance)
        
        if len(matches) < 10:
            return {'confidence': 0.0, 'position': None}
        
        # Get matched keypoint pairs
        src_pts = np.float32([aerial['keypoints'][m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
        dst_pts = np.float32([satellite['keypoints'][m.trainIdx]['pixel'] for m in matches]).reshape(-1, 1, 2)
        
        # Find homography with RANSAC
        H, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
        
        if H is None:
            return {'confidence': 0.0, 'position': None}
        
        inliers = mask.sum()
        confidence = inliers / len(matches)
        
        # Get position from center of aerial image in satellite coordinates
        center = np.array([[[aerial['image_shape'][1] / 2, aerial['image_shape'][0] / 2]]], dtype=np.float32)
        transformed_center = cv2.perspectiveTransform(center, H)[0, 0]
        
        # Find closest satellite keypoint to get geo-coordinates
        min_dist = float('inf')
        matched_geo = None
        for kp in satellite['keypoints']:
            dist = np.linalg.norm(np.array(kp['pixel']) - transformed_center)
            if dist < min_dist:
                min_dist = dist
                matched_geo = kp['geo']
        
        return {
            'confidence': confidence,
            'position': matched_geo,
            'inliers': inliers,
            'total_matches': len(matches)
        }
    
    def _publish_matched_pose(self, match_result: dict, stamp):
        """Publish matched position as PoseWithCovarianceStamped."""
        lon, lat = match_result['position']
        
        # Convert to local frame (UTM)
        x, y = self.transformer.transform(lat, lon)
        
        # Subtract initial position to get local coordinates
        init_x, init_y = self.transformer.transform(
            self.initial_coords[0], 
            self.initial_coords[1]
        )
        
        local_x = x - init_x
        local_y = y - init_y
        
        # Create message
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = 'map'
        
        pose_msg.pose.pose.position.x = local_x
        pose_msg.pose.pose.position.y = local_y
        pose_msg.pose.pose.position.z = self.current_altitude
        
        # Covariance based on confidence
        # Lower confidence = higher uncertainty
        position_var = 10.0 / max(match_result['confidence'], 0.1)  # meters^2
        pose_msg.pose.covariance = [
            position_var, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, position_var, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 100.0, 0.0, 0.0, 0.0,  # High Z uncertainty (no altitude from matching)
            0.0, 0.0, 0.0, 999.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 999.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 999.0,
        ]
        
        self.match_pose_pub.publish(pose_msg)
        
        # Publish confidence
        conf_msg = Float32()
        conf_msg.data = match_result['confidence']
        self.confidence_pub.publish(conf_msg)
        
        self.get_logger().info(
            f"Satellite match: ({local_x:.1f}, {local_y:.1f})m, "
            f"confidence: {match_result['confidence']:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = SatelliteMatcherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## Fusion Layer

### VIO + Satellite Fusion EKF

```python
# src/state_fusion/nodes/fusion_node.py
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from std_msgs.msg import Float32
import numpy as np
from scipy.spatial.transform import Rotation

class FusionNode(Node):
    """Fuse VIO and satellite matching for global pose."""
    
    def __init__(self):
        super().__init__('fusion_node')
        
        # State vector: [x, y, z, vx, vy, vz, qw, qx, qy, qz, T_vio_global (6 DOF)]
        # Simplified to position + orientation + VIO-to-global transform
        self.state = np.zeros(16)
        self.state[6] = 1.0  # qw = 1 (identity quaternion)
        
        # Covariance
        self.P = np.eye(16) * 100.0  # High initial uncertainty
        
        # Process noise
        self.Q = np.eye(16) * 0.01
        
        # VIO-to-global alignment
        self.vio_to_global_initialized = False
        self.vio_origin = None
        
        # Subscriptions
        self.vio_sub = self.create_subscription(
            Odometry,
            '/vio/odom',
            self.vio_callback,
            10
        )
        
        self.sat_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/sat_match/matched_pose',
            self.satellite_callback,
            10
        )
        
        # Publishers
        self.fused_odom_pub = self.create_publisher(
            Odometry,
            '/state/fused_odom',
            10
        )
        
        self.vision_pose_pub = self.create_publisher(
            PoseStamped,
            '/mavros/vision_pose/pose',
            10
        )
        
        # Timer for prediction step
        self.last_update = self.get_clock().now()
        self.create_timer(0.02, self.prediction_step)  # 50Hz
        
        self.get_logger().info('Fusion node initialized')
        
    def vio_callback(self, msg: Odometry):
        """Handle VIO odometry updates."""
        if not self.vio_to_global_initialized:
            # Store first VIO pose as origin
            self.vio_origin = msg.pose.pose
            self.vio_to_global_initialized = True
            self.get_logger().info('VIO origin initialized')
            return
        
        # Extract relative motion from VIO origin
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
        
        # Update state with VIO (as prediction)
        # Transform VIO to global frame using estimated transform
        self._update_from_vio(vio_pos, vio_quat, vio_vel, msg.pose.covariance)
        
        # Publish fused state
        self._publish_fused_state(msg.header.stamp)
        
    def satellite_callback(self, msg: PoseWithCovarianceStamped):
        """Handle satellite matching updates (absolute position)."""
        sat_pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])
        
        sat_cov = np.array(msg.pose.covariance).reshape(6, 6)[:3, :3]
        
        # EKF update with satellite measurement
        self._update_from_satellite(sat_pos, sat_cov)
        
        self.get_logger().info(f'Satellite update applied, pos: {sat_pos}')
        
    def prediction_step(self):
        """Prediction step with constant velocity model."""
        now = self.get_clock().now()
        dt = (now - self.last_update).nanoseconds / 1e9
        self.last_update = now
        
        if dt > 0.1:  # Skip if too long
            return
        
        # State prediction: x = x + v*dt
        self.state[0:3] += self.state[3:6] * dt
        
        # Covariance prediction
        F = np.eye(16)
        F[0, 3] = dt
        F[1, 4] = dt
        F[2, 5] = dt
        
        self.P = F @ self.P @ F.T + self.Q * dt
        
    def _update_from_vio(self, pos, quat, vel, cov):
        """Update state from VIO measurement."""
        # VIO provides relative motion, integrate into global frame
        # Using VIO-to-global transform from state[10:16]
        
        # Simplified: direct copy with transform
        # In full implementation, apply rotation and translation from state
        T_vio_global_pos = self.state[10:13]
        T_vio_global_rot = Rotation.from_euler('xyz', self.state[13:16])
        
        # Transform VIO position to global
        global_pos = T_vio_global_rot.apply(pos) + T_vio_global_pos
        global_vel = T_vio_global_rot.apply(vel)
        
        # Update state
        self.state[0:3] = global_pos
        self.state[3:6] = global_vel
        self.state[6:10] = quat
        
    def _update_from_satellite(self, pos, cov):
        """EKF update from satellite absolute position."""
        # Measurement model: z = H*x
        H = np.zeros((3, 16))
        H[0, 0] = 1.0
        H[1, 1] = 1.0
        H[2, 2] = 1.0
        
        # Innovation
        y = pos - H @ self.state
        
        # Innovation covariance
        R = cov
        S = H @ self.P @ H.T + R
        
        # Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # State update
        self.state = self.state + K @ y
        
        # Covariance update
        self.P = (np.eye(16) - K @ H) @ self.P
        
        # Also update VIO-to-global transform to align VIO with satellite
        # This corrects VIO drift over time
        self._update_vio_alignment(pos)
        
    def _update_vio_alignment(self, sat_pos):
        """Update VIO-to-global transform based on satellite fix."""
        # Compute correction needed
        vio_predicted_pos = self.state[0:3]  # Current VIO estimate in global
        correction = sat_pos - vio_predicted_pos
        
        # Gradually update transform (low-pass filter)
        alpha = 0.1  # Learning rate
        self.state[10:13] += alpha * correction
        
    def _publish_fused_state(self, stamp):
        """Publish fused state to MAVROS and ROS2."""
        # Odometry message
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'base_link'
        
        odom.pose.pose.position.x = self.state[0]
        odom.pose.pose.position.y = self.state[1]
        odom.pose.pose.position.z = self.state[2]
        
        odom.pose.pose.orientation.w = self.state[6]
        odom.pose.pose.orientation.x = self.state[7]
        odom.pose.pose.orientation.y = self.state[8]
        odom.pose.pose.orientation.z = self.state[9]
        
        odom.twist.twist.linear.x = self.state[3]
        odom.twist.twist.linear.y = self.state[4]
        odom.twist.twist.linear.z = self.state[5]
        
        # Copy covariance
        odom.pose.covariance = list(self.P[:6, :6].flatten())
        
        self.fused_odom_pub.publish(odom)
        
        # Vision pose for PX4
        vision_pose = PoseStamped()
        vision_pose.header = odom.header
        vision_pose.pose = odom.pose.pose
        
        self.vision_pose_pub.publish(vision_pose)


def main(args=None):
    rclpy.init(args=args)
    node = FusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## PX4 Integration

### EKF2 Configuration for Vision Fusion

```yaml
# config/px4/ekf2_params.yaml
# Parameters to set on PX4 for vision position fusion

EKF2_EV_CTRL: 15          # Enable vision position, velocity, and yaw fusion
EKF2_EVP_NOISE: 0.1       # Vision position noise (meters)
EKF2_EVV_NOISE: 0.1       # Vision velocity noise (m/s)
EKF2_EVA_NOISE: 0.1       # Vision yaw angle noise (rad)
EKF2_EV_DELAY: 50         # Vision delay (ms)
EKF2_HGT_REF: 3           # Height reference: 0=baro, 1=GPS, 2=range, 3=vision

# Disable GPS if in GPS-denied mode
EKF2_GPS_CTRL: 0          # Disable GPS fusion
EKF2_REQ_GPS_H: 0         # Don't require GPS for initialization

# Fusion source
EKF2_AID_MASK: 24         # Use vision position and yaw (bits 3 and 4)
```

### MAVROS2 Vision Pose Bridge

```yaml
# config/mavros/mavros_config.yaml
/**:
  ros__parameters:
    # Connection
    fcu_url: "/dev/ttyTHS1:921600"  # Orin UART to Pixhawk
    gcs_url: "udp://:14550@"
    
    # Vision pose
    local_position:
      frame_id: "map"
      tf:
        send: true
        frame_id: "map"
        child_frame_id: "base_link"
    
    # Time sync
    timesync:
      mode: "MAVLINK"
      
    # Vision pose input
    vision_pose:
      frame_id: "map"
```

---

## Calibration Procedures

### 1. Camera Intrinsics Calibration

```bash
# Use ROS2 camera_calibration
ros2 run camera_calibration cameracalibrator \
    --size 9x6 \
    --square 0.025 \
    --camera /vio/camera/image_raw
```

### 2. Camera-IMU Extrinsics Calibration

```bash
# Use Kalibr
rosrun kalibr kalibr_calibrate_imu_camera \
    --target april_6x6.yaml \
    --cam cam.yaml \
    --imu imu.yaml \
    --bag calib.bag
```

### 3. VIO-to-Satellite Alignment

Performed automatically during flight using the fusion node, but can be initialized manually:

```yaml
# config/mission/initial_pose.yaml
initial_position:
  latitude: 39.56789
  longitude: -76.12345
  altitude: 0.0  # AGL
  heading: 90.0  # degrees from north

vio_to_global:
  # Initial estimate (refined during flight)
  translation: [0.0, 0.0, 0.0]
  rotation_euler_xyz: [0.0, 0.0, 0.0]  # radians
```

---

## Performance Considerations

### Computational Budget (Orin Nano)

| Component | Frequency | CPU % | GPU % | Memory |
|-----------|-----------|-------|-------|--------|
| OpenVINS VIO | 30 Hz | 40% | 10% | 500MB |
| Satellite Matching | 1 Hz | 20% | 40% | 1GB |
| Fusion Node | 50 Hz | 5% | 0% | 50MB |
| MAVROS | 50 Hz | 10% | 0% | 100MB |
| **Total** | - | **~75%** | **~50%** | **~1.7GB** |

### Optimization Strategies

1. **Feature Extraction on GPU** - Use CUDA for ORB/SuperPoint
2. **Satellite Feature Precomputation** - Pre-extract features offline
3. **Hierarchical Matching** - Coarse-to-fine search
4. **Altitude-based Filtering** - Only search relevant scales
5. **Region Partitioning** - Spatial index for large areas

---

## Testing & Validation

### Unit Tests

```bash
# Run VIO unit tests
ros2 test vio_core --verbose

# Run satellite matching tests
ros2 test satellite_matching --verbose
```

### Integration Tests

```bash
# Replay rosbag with simulated satellite matching
ros2 launch integration_tests vio_sat_fusion_test.launch.py \
    bag_file:=test_flight.db3 \
    satellite_data:=test_region/
```

### Accuracy Metrics

- **VIO-only drift**: Expected < 1% of distance traveled
- **Satellite correction interval**: 1-2 seconds
- **Fused position error**: Target < 5m RMSE
- **Heading error**: Target < 3° RMSE
