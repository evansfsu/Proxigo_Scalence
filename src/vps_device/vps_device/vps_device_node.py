"""
ROS2 node: subscribe to image + altitude, run VPS estimator, publish pose + confidence.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Float64
from geometry_msgs.msg import PoseWithCovarianceStamped

from cv_bridge import CvBridge

from .config import VPSDeviceConfig
from .estimator import VPSEstimator, create_estimator_from_proxigo_region
from pathlib import Path


class VPSDeviceNode(Node):
    """ROS2 node wrapping VPSEstimator: image + altitude -> position estimate."""

    def __init__(self):
        super().__init__("vps_device_node")

        self.declare_parameter("reference_path", "")
        self.declare_parameter("image_topic", "/vio/camera/image_raw")
        self.declare_parameter("altitude_topic", "/mavros/global_position/rel_alt")
        self.declare_parameter("publish_topic", "/vps_device/position")
        self.declare_parameter("confidence_topic", "/vps_device/confidence")
        self.declare_parameter("match_interval_sec", 1.0)
        self.declare_parameter("fov_h", 71.5)
        self.declare_parameter("fov_d", 79.5)
        self.declare_parameter("width_px", 1920)
        self.declare_parameter("height_px", 1080)

        ref_path = self.get_parameter("reference_path").value
        if not ref_path:
            self.get_logger().error("reference_path parameter is required")
            raise ValueError("reference_path must be set")
        ref_path = Path(ref_path)
        if not ref_path.is_dir():
            self.get_logger().error(f"reference_path is not a directory: {ref_path}")
            raise ValueError("reference_path must be a Proxigo region directory")
        config = VPSDeviceConfig(
            h_fov_deg=self.get_parameter("fov_h").value,
            d_fov_deg=self.get_parameter("fov_d").value,
            width_px=self.get_parameter("width_px").value,
            height_px=self.get_parameter("height_px").value,
        )
        self.estimator = create_estimator_from_proxigo_region(ref_path, config)
        self.bridge = CvBridge()
        self.last_image = None
        self.last_image_stamp = None
        self.current_altitude = 100.0
        self.match_interval_sec = self.get_parameter("match_interval_sec").value
        self.last_match_time = self.get_clock().now()

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.image_sub = self.create_subscription(
            Image,
            self.get_parameter("image_topic").value,
            self.image_callback,
            sensor_qos,
        )
        self.altitude_sub = self.create_subscription(
            Float64,
            self.get_parameter("altitude_topic").value,
            self.altitude_callback,
            10,
        )
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            self.get_parameter("publish_topic").value,
            10,
        )
        self.confidence_pub = self.create_publisher(
            Float32,
            self.get_parameter("confidence_topic").value,
            10,
        )
        self.get_logger().info("VPS device node started")

    def image_callback(self, msg: Image) -> None:
        self.last_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.last_image_stamp = msg.header.stamp

    def altitude_callback(self, msg: Float64) -> None:
        self.current_altitude = msg.data

    def timer_callback(self) -> None:
        if self.last_image is None:
            return
        now = self.get_clock().now()
        dt = (now - self.last_match_time).nanoseconds / 1e9
        if dt < self.match_interval_sec:
            return
        self.last_match_time = now
        result = self.estimator.estimate(
            self.last_image,
            self.current_altitude,
            last_lat_lon=None,
        )
        stamp = self.last_image_stamp if self.last_image_stamp else now.to_msg()
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = "map"
        pose_msg.pose.pose.position.x = float(result.lat)
        pose_msg.pose.pose.position.y = float(result.lon)
        pose_msg.pose.pose.position.z = self.current_altitude
        pose_msg.pose.pose.orientation.w = 1.0
        var = 10.0 / max(result.confidence, 0.1)
        pose_msg.pose.covariance = [
            var, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, var, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 100.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 999.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 999.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 999.0,
        ]
        self.pose_pub.publish(pose_msg)
        conf_msg = Float32()
        conf_msg.data = result.confidence
        self.confidence_pub.publish(conf_msg)
        if result.success:
            self.get_logger().info(
                f"VPS lat={result.lat:.6f} lon={result.lon:.6f} conf={result.confidence:.2f}"
            )


def main(args=None):
    rclpy.init(args=args)
    try:
        node = VPSDeviceNode()
        timer = node.create_timer(0.2, node.timer_callback)  # 5 Hz check
        rclpy.spin(node)
    except ValueError as e:
        rclpy.logging.get_logger("vps_device_node").error(str(e))
        raise
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
