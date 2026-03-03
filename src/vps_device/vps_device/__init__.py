"""
VPS Device - Visual Positioning System device module.

Estimates position by comparing live imaging and altitude data to
downloaded satellite reference imagery. Based on UAVs@Berkeley fa24_106a
GPS-denied navigation approach. Can run standalone (no ROS2) or as a ROS2 node.
"""

from vps_device.estimator import VPSEstimator
from vps_device.config import VPSDeviceConfig

__all__ = ['VPSEstimator', 'VPSDeviceConfig']
