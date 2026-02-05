#!/usr/bin/env python3
"""
Region Manager Node

Manages satellite imagery regions and provides region switching based on position.

Services:
    /sat_match/load_region (std_srvs/SetString) - Load a specific region
    /sat_match/get_region_info (std_srvs/Trigger) - Get current region info

Publications:
    /sat_match/region_status (std_msgs/String) - Current region status
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import json
from pathlib import Path
from typing import Dict, List, Optional


class RegionManagerNode(Node):
    """Manages satellite imagery regions."""

    def __init__(self):
        super().__init__('region_manager_node')

        # Parameters
        self.declare_parameter('satellite_data_path', '/satellite_data')
        self.declare_parameter('auto_switch_regions', True)
        self.declare_parameter('region_overlap_margin', 100.0)  # meters

        self.sat_data_path = Path(self.get_parameter('satellite_data_path').value)
        self.auto_switch = self.get_parameter('auto_switch_regions').value
        self.overlap_margin = self.get_parameter('region_overlap_margin').value

        # State
        self.available_regions: Dict[str, dict] = {}
        self.current_region_id: Optional[str] = None
        self.current_position = None

        # Scan available regions
        self.scan_regions()

        # Publishers
        self.status_pub = self.create_publisher(String, '/sat_match/region_status', 10)

        # Subscriptions
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/vio/pose',
            self.pose_callback,
            10
        )

        # Timer for status updates
        self.create_timer(5.0, self.publish_status)

        self.get_logger().info(f'Region manager initialized with {len(self.available_regions)} regions')

    def scan_regions(self):
        """Scan for available satellite regions."""
        regions_path = self.sat_data_path / 'regions'
        
        if not regions_path.exists():
            self.get_logger().warn(f'Regions path does not exist: {regions_path}')
            return

        for region_dir in regions_path.iterdir():
            if region_dir.is_dir():
                metadata_path = region_dir / 'metadata.json'
                if metadata_path.exists():
                    try:
                        with open(metadata_path, 'r') as f:
                            metadata = json.load(f)
                        
                        region_id = region_dir.name
                        self.available_regions[region_id] = metadata
                        
                        self.get_logger().info(
                            f'Found region: {region_id} - {metadata.get("name", "unnamed")}'
                        )
                    except Exception as e:
                        self.get_logger().error(f'Error loading region {region_dir.name}: {e}')

    def pose_callback(self, msg: PoseStamped):
        """Update current position."""
        self.current_position = (
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        )

        # Auto-switch regions if enabled
        if self.auto_switch:
            self.check_region_switch()

    def check_region_switch(self):
        """Check if we should switch to a different region."""
        # Currently just use the first available region
        # In a full implementation, this would check geographic bounds
        if self.current_region_id is None and self.available_regions:
            self.current_region_id = list(self.available_regions.keys())[0]
            self.get_logger().info(f'Auto-selected region: {self.current_region_id}')

    def publish_status(self):
        """Publish current region status."""
        status = {
            'current_region': self.current_region_id,
            'available_regions': list(self.available_regions.keys()),
            'position': self.current_position
        }

        status_msg = String()
        status_msg.data = json.dumps(status)
        self.status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    node = RegionManagerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
