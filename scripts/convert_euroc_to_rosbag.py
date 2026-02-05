#!/usr/bin/env python3
"""
Convert EuRoC MAV Dataset to ROS2 Bag

This script converts EuRoC MAV format datasets to ROS2 bag files for testing.
Run this inside the Docker container where ROS2 is available.

Usage:
    python3 scripts/convert_euroc_to_rosbag.py test_data/MH_01_easy test_data/mh01.db3
"""

import argparse
import csv
import os
import sys
from pathlib import Path
from datetime import datetime

# Check if we're in a ROS2 environment
try:
    import rclpy
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import Image, Imu, CameraInfo
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import Header
    import rosbag2_py
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False
    print("Warning: ROS2 not available. Run this script inside the Docker container.")

try:
    import cv2
    import numpy as np
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False


def read_imu_data(imu_path: Path) -> list:
    """Read IMU data from EuRoC CSV format."""
    imu_data = []
    with open(imu_path, 'r') as f:
        reader = csv.reader(f)
        next(reader)  # Skip header
        for row in reader:
            timestamp_ns = int(row[0])
            wx, wy, wz = float(row[1]), float(row[2]), float(row[3])  # Angular velocity
            ax, ay, az = float(row[4]), float(row[5]), float(row[6])  # Linear acceleration
            imu_data.append({
                'timestamp_ns': timestamp_ns,
                'angular_velocity': (wx, wy, wz),
                'linear_acceleration': (ax, ay, az)
            })
    return imu_data


def read_image_timestamps(images_path: Path) -> list:
    """Read image timestamps from EuRoC CSV format."""
    timestamps = []
    csv_path = images_path / 'data.csv'
    if csv_path.exists():
        with open(csv_path, 'r') as f:
            reader = csv.reader(f)
            next(reader)  # Skip header
            for row in reader:
                timestamp_ns = int(row[0])
                filename = row[1].strip()
                timestamps.append({
                    'timestamp_ns': timestamp_ns,
                    'filename': filename
                })
    return timestamps


def read_groundtruth(gt_path: Path) -> list:
    """Read ground truth poses from EuRoC CSV format."""
    poses = []
    with open(gt_path, 'r') as f:
        reader = csv.reader(f)
        next(reader)  # Skip header
        for row in reader:
            timestamp_ns = int(row[0])
            px, py, pz = float(row[1]), float(row[2]), float(row[3])
            qw, qx, qy, qz = float(row[4]), float(row[5]), float(row[6]), float(row[7])
            poses.append({
                'timestamp_ns': timestamp_ns,
                'position': (px, py, pz),
                'orientation': (qw, qx, qy, qz)
            })
    return poses


def convert_to_rosbag(euroc_path: Path, output_path: Path):
    """Convert EuRoC dataset to ROS2 bag."""
    if not HAS_ROS2:
        print("ERROR: ROS2 not available. Run inside Docker container.")
        sys.exit(1)

    if not HAS_CV2:
        print("ERROR: OpenCV not available. Install opencv-python.")
        sys.exit(1)

    print(f"Converting: {euroc_path}")
    print(f"Output: {output_path}")
    print()

    # Check paths
    mav0_path = euroc_path / 'mav0'
    if not mav0_path.exists():
        print(f"ERROR: mav0 directory not found in {euroc_path}")
        sys.exit(1)

    # Read data
    print("Reading IMU data...")
    imu_path = mav0_path / 'imu0' / 'data.csv'
    imu_data = read_imu_data(imu_path) if imu_path.exists() else []
    print(f"  Found {len(imu_data)} IMU samples")

    print("Reading camera timestamps...")
    cam0_path = mav0_path / 'cam0'
    cam_timestamps = read_image_timestamps(cam0_path)
    print(f"  Found {len(cam_timestamps)} images")

    print("Reading ground truth...")
    gt_path = mav0_path / 'state_groundtruth_estimate0' / 'data.csv'
    gt_data = read_groundtruth(gt_path) if gt_path.exists() else []
    print(f"  Found {len(gt_data)} ground truth poses")

    # Create bag writer
    print()
    print("Creating ROS2 bag...")
    
    writer = rosbag2_py.SequentialWriter()
    storage_options = rosbag2_py.StorageOptions(
        uri=str(output_path),
        storage_id='sqlite3'
    )
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )
    writer.open(storage_options, converter_options)

    # Create topics
    imu_topic = rosbag2_py.TopicMetadata(
        name='/vio/imu/data',
        type='sensor_msgs/msg/Imu',
        serialization_format='cdr'
    )
    writer.create_topic(imu_topic)

    image_topic = rosbag2_py.TopicMetadata(
        name='/vio/camera/image_raw',
        type='sensor_msgs/msg/Image',
        serialization_format='cdr'
    )
    writer.create_topic(image_topic)

    gt_topic = rosbag2_py.TopicMetadata(
        name='/groundtruth/pose',
        type='geometry_msgs/msg/PoseStamped',
        serialization_format='cdr'
    )
    writer.create_topic(gt_topic)

    # Write IMU data
    print("Writing IMU data...")
    for imu in imu_data:
        msg = Imu()
        msg.header.stamp.sec = imu['timestamp_ns'] // 1000000000
        msg.header.stamp.nanosec = imu['timestamp_ns'] % 1000000000
        msg.header.frame_id = 'imu_link'
        
        msg.angular_velocity.x = imu['angular_velocity'][0]
        msg.angular_velocity.y = imu['angular_velocity'][1]
        msg.angular_velocity.z = imu['angular_velocity'][2]
        
        msg.linear_acceleration.x = imu['linear_acceleration'][0]
        msg.linear_acceleration.y = imu['linear_acceleration'][1]
        msg.linear_acceleration.z = imu['linear_acceleration'][2]

        writer.write('/vio/imu/data', serialize_message(msg), imu['timestamp_ns'])

    # Write images
    print("Writing images...")
    data_path = cam0_path / 'data'
    for i, cam in enumerate(cam_timestamps):
        img_path = data_path / cam['filename']
        if img_path.exists():
            img = cv2.imread(str(img_path), cv2.IMREAD_GRAYSCALE)
            
            msg = Image()
            msg.header.stamp.sec = cam['timestamp_ns'] // 1000000000
            msg.header.stamp.nanosec = cam['timestamp_ns'] % 1000000000
            msg.header.frame_id = 'camera_optical_frame'
            msg.height = img.shape[0]
            msg.width = img.shape[1]
            msg.encoding = 'mono8'
            msg.step = img.shape[1]
            msg.data = img.tobytes()

            writer.write('/vio/camera/image_raw', serialize_message(msg), cam['timestamp_ns'])

        if (i + 1) % 100 == 0:
            print(f"  Processed {i + 1}/{len(cam_timestamps)} images")

    # Write ground truth
    print("Writing ground truth...")
    for gt in gt_data:
        msg = PoseStamped()
        msg.header.stamp.sec = gt['timestamp_ns'] // 1000000000
        msg.header.stamp.nanosec = gt['timestamp_ns'] % 1000000000
        msg.header.frame_id = 'map'
        
        msg.pose.position.x = gt['position'][0]
        msg.pose.position.y = gt['position'][1]
        msg.pose.position.z = gt['position'][2]
        
        msg.pose.orientation.w = gt['orientation'][0]
        msg.pose.orientation.x = gt['orientation'][1]
        msg.pose.orientation.y = gt['orientation'][2]
        msg.pose.orientation.z = gt['orientation'][3]

        writer.write('/groundtruth/pose', serialize_message(msg), gt['timestamp_ns'])

    print()
    print("Conversion complete!")
    print(f"Output: {output_path}")


def main():
    parser = argparse.ArgumentParser(
        description='Convert EuRoC MAV dataset to ROS2 bag'
    )
    parser.add_argument(
        'input_path',
        type=str,
        help='Path to EuRoC dataset directory (containing mav0/)'
    )
    parser.add_argument(
        'output_path',
        type=str,
        help='Output ROS2 bag path'
    )

    args = parser.parse_args()

    convert_to_rosbag(Path(args.input_path), Path(args.output_path))


if __name__ == '__main__':
    main()
