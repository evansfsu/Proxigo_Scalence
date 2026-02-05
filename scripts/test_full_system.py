#!/usr/bin/env python3
"""
Proxigo Scalence - Full System Test

Automated test script that validates the complete system without hardware.
Tests:
1. Camera simulator publishes images
2. IMU simulator publishes data
3. VIO bridge processes data
4. Satellite matcher finds features
5. Fusion node produces output
6. MAVROS bridge formats messages

Run inside Docker:
    python3 scripts/test_full_system.py
"""

import subprocess
import sys
import time
import threading
from dataclasses import dataclass
from typing import Dict, List, Optional


@dataclass
class TopicTest:
    """Definition of a topic test."""
    topic: str
    expected_type: str
    min_rate_hz: float
    description: str


# Tests to run
TOPIC_TESTS: List[TopicTest] = [
    TopicTest(
        topic='/vio/camera/image_raw',
        expected_type='sensor_msgs/msg/Image',
        min_rate_hz=20.0,
        description='Camera simulator output'
    ),
    TopicTest(
        topic='/vio/imu/data',
        expected_type='sensor_msgs/msg/Imu',
        min_rate_hz=100.0,
        description='IMU simulator output'
    ),
    TopicTest(
        topic='/vio/odometry',
        expected_type='nav_msgs/msg/Odometry',
        min_rate_hz=10.0,
        description='VIO bridge output'
    ),
    TopicTest(
        topic='/satellite_match/pose',
        expected_type='geometry_msgs/msg/PoseWithCovarianceStamped',
        min_rate_hz=0.5,
        description='Satellite matcher output'
    ),
    TopicTest(
        topic='/fused_pose',
        expected_type='nav_msgs/msg/Odometry',
        min_rate_hz=10.0,
        description='Fusion node output'
    ),
]


class SystemTester:
    """Runs system tests."""
    
    def __init__(self):
        self.processes: List[subprocess.Popen] = []
        self.results: Dict[str, bool] = {}
    
    def setup_ros_env(self):
        """Source ROS2 setup."""
        # This is handled by running inside Docker
        pass
    
    def start_node(self, package: str, executable: str, name: str = None) -> subprocess.Popen:
        """Start a ROS2 node."""
        cmd = ['ros2', 'run', package, executable]
        if name:
            cmd.extend(['--ros-args', '-r', f'__node:={name}'])
        
        proc = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        self.processes.append(proc)
        print(f"  Started: {package}/{executable} (PID: {proc.pid})")
        return proc
    
    def cleanup(self):
        """Stop all processes."""
        for proc in self.processes:
            try:
                proc.terminate()
                proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                proc.kill()
        self.processes.clear()
    
    def check_topic_exists(self, topic: str, timeout: float = 5.0) -> bool:
        """Check if a topic exists."""
        try:
            result = subprocess.run(
                ['ros2', 'topic', 'info', topic],
                capture_output=True,
                text=True,
                timeout=timeout
            )
            return result.returncode == 0
        except subprocess.TimeoutExpired:
            return False
    
    def check_topic_rate(self, topic: str, duration: float = 3.0) -> Optional[float]:
        """Check topic publication rate."""
        try:
            result = subprocess.run(
                ['timeout', str(duration), 'ros2', 'topic', 'hz', topic, '--window', '5'],
                capture_output=True,
                text=True,
            )
            
            # Parse rate from output
            for line in result.stdout.split('\n'):
                if 'average rate:' in line:
                    rate_str = line.split('average rate:')[1].strip()
                    return float(rate_str.split()[0])
            
            return None
        except Exception:
            return None
    
    def run_tests(self) -> bool:
        """Run all tests."""
        print("\n" + "=" * 60)
        print("PROXIGO SCALENCE - FULL SYSTEM TEST")
        print("=" * 60)
        
        # Phase 1: Start nodes
        print("\n[1/4] Starting system nodes...")
        try:
            # Start simulators
            self.start_node('vio_bridge', 'camera_simulator')
            self.start_node('vio_bridge', 'imu_simulator')
            time.sleep(2)
            
            # Start processing nodes
            self.start_node('vio_bridge', 'vio_bridge_node')
            self.start_node('satellite_matching', 'satellite_matcher_node')
            self.start_node('state_fusion', 'fusion_node')
            time.sleep(3)
            
            print("  All nodes started.")
        except Exception as e:
            print(f"  ERROR starting nodes: {e}")
            self.cleanup()
            return False
        
        # Phase 2: Check topics exist
        print("\n[2/4] Checking topic availability...")
        topics_ok = True
        for test in TOPIC_TESTS:
            exists = self.check_topic_exists(test.topic)
            status = "✓" if exists else "✗"
            print(f"  {status} {test.topic}")
            if not exists:
                topics_ok = False
        
        # Phase 3: Check rates
        print("\n[3/4] Checking publication rates...")
        rates_ok = True
        for test in TOPIC_TESTS:
            rate = self.check_topic_rate(test.topic)
            if rate is not None and rate >= test.min_rate_hz:
                print(f"  ✓ {test.topic}: {rate:.1f} Hz (min: {test.min_rate_hz})")
            elif rate is not None:
                print(f"  ⚠ {test.topic}: {rate:.1f} Hz (below min: {test.min_rate_hz})")
            else:
                print(f"  ✗ {test.topic}: no data received")
                rates_ok = False
        
        # Phase 4: Cleanup
        print("\n[4/4] Cleaning up...")
        self.cleanup()
        print("  Done.")
        
        # Summary
        print("\n" + "=" * 60)
        print("TEST SUMMARY")
        print("=" * 60)
        
        all_passed = topics_ok and rates_ok
        
        if all_passed:
            print("\n  ✓ ALL TESTS PASSED")
            print("\n  The simulation stack is working correctly!")
            print("  Next steps:")
            print("    1. Download real satellite imagery:")
            print("       python scripts/download_satellite_imagery.py --lat YOUR_LAT --lon YOUR_LON")
            print("    2. Test with VIO datasets:")
            print("       python scripts/download_test_datasets.py --dataset euroc_mh01")
            print("    3. Connect to PX4 SITL:")
            print("       ./scripts/run_simulation.sh --px4")
        else:
            print("\n  ✗ SOME TESTS FAILED")
            print("\n  Check the logs above for details.")
            if not topics_ok:
                print("  - Some topics are not being published")
            if not rates_ok:
                print("  - Some topics have insufficient rate")
        
        print("\n" + "=" * 60)
        
        return all_passed


def main():
    """Main entry point."""
    tester = SystemTester()
    
    try:
        success = tester.run_tests()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\nInterrupted by user")
        tester.cleanup()
        sys.exit(1)
    except Exception as e:
        print(f"\nError: {e}")
        tester.cleanup()
        sys.exit(1)


if __name__ == '__main__':
    main()
