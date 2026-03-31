"""
VPS Device - Visual Positioning System device module.

Estimates position by comparing live imaging and altitude data to
downloaded satellite reference imagery. Can run standalone (no ROS2) or as a ROS2 node.
"""

from vps_device.config import VPSDeviceConfig
from vps_device.ekf import VPSFusionEKF, EKFConfig
from vps_device.estimator import VPSEstimator
from vps_device.visual_odometry import VisualOdometry, HeadingCalibrator, VOResult

__all__ = ['VPSEstimator', 'VPSDeviceConfig', 'VPSFusionEKF', 'EKFConfig',
           'VisualOdometry', 'HeadingCalibrator', 'VOResult']
