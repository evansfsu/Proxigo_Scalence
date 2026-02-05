#!/usr/bin/env python3
"""
Proxigo Scalence - Download VIO Test Datasets

Downloads popular VIO benchmark datasets for testing and development.
Supports EuRoC MAV and TUM-VI datasets.

Usage:
    python scripts/download_test_datasets.py euroc     # Download EuRoC MH_01
    python scripts/download_test_datasets.py tumvi     # Download TUM-VI room1
    python scripts/download_test_datasets.py all       # Download all

Datasets are saved to test_data/ directory.
"""

import argparse
import os
import sys
import urllib.request
import zipfile
import tarfile
from pathlib import Path
import hashlib


# Dataset URLs and info
DATASETS = {
    'euroc_mh01': {
        'name': 'EuRoC MAV MH_01_easy',
        'url': 'http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/machine_hall/MH_01_easy/MH_01_easy.zip',
        'filename': 'MH_01_easy.zip',
        'extracted_dir': 'MH_01_easy',
        'size_mb': 1600,
        'description': 'Indoor machine hall sequence, easy difficulty',
    },
    'euroc_v101': {
        'name': 'EuRoC MAV V1_01_easy',
        'url': 'http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/vicon_room1/V1_01_easy/V1_01_easy.zip',
        'filename': 'V1_01_easy.zip',
        'extracted_dir': 'V1_01_easy',
        'size_mb': 800,
        'description': 'Vicon room sequence, easy difficulty',
    },
    'tumvi_room1': {
        'name': 'TUM-VI room1',
        'url': 'https://vision.in.tum.de/tumvi/exported/euroc/512_16/dataset-room1_512_16.tar',
        'filename': 'dataset-room1_512_16.tar',
        'extracted_dir': 'dataset-room1_512_16',
        'size_mb': 2500,
        'description': 'TUM VI dataset room1 sequence (EuRoC format)',
    },
}


def download_with_progress(url: str, dest_path: Path, expected_size_mb: int = 0):
    """Download file with progress bar."""
    print(f"Downloading: {url}")
    print(f"Destination: {dest_path}")
    if expected_size_mb > 0:
        print(f"Expected size: ~{expected_size_mb} MB")
    print()

    def progress_hook(count, block_size, total_size):
        if total_size > 0:
            percent = min(100, count * block_size * 100 // total_size)
            downloaded_mb = count * block_size / (1024 * 1024)
            total_mb = total_size / (1024 * 1024)
            bar_length = 40
            filled = int(bar_length * percent / 100)
            bar = '=' * filled + '-' * (bar_length - filled)
            print(f'\r[{bar}] {percent}% ({downloaded_mb:.1f}/{total_mb:.1f} MB)', end='', flush=True)

    try:
        urllib.request.urlretrieve(url, dest_path, progress_hook)
        print("\nDownload complete!")
        return True
    except Exception as e:
        print(f"\nDownload failed: {e}")
        return False


def extract_archive(archive_path: Path, dest_dir: Path):
    """Extract zip or tar archive."""
    print(f"Extracting: {archive_path}")
    print(f"Destination: {dest_dir}")

    if archive_path.suffix == '.zip':
        with zipfile.ZipFile(archive_path, 'r') as zf:
            zf.extractall(dest_dir)
    elif archive_path.suffix in ['.tar', '.gz', '.tgz']:
        mode = 'r:gz' if '.gz' in archive_path.suffixes else 'r'
        with tarfile.open(archive_path, mode) as tf:
            tf.extractall(dest_dir)
    else:
        print(f"Unknown archive format: {archive_path.suffix}")
        return False

    print("Extraction complete!")
    return True


def download_dataset(dataset_id: str, output_dir: Path, keep_archive: bool = False):
    """Download and extract a dataset."""
    if dataset_id not in DATASETS:
        print(f"Unknown dataset: {dataset_id}")
        print(f"Available datasets: {list(DATASETS.keys())}")
        return False

    dataset = DATASETS[dataset_id]
    print("=" * 60)
    print(f"Dataset: {dataset['name']}")
    print(f"Description: {dataset['description']}")
    print("=" * 60)
    print()

    # Create output directory
    output_dir.mkdir(parents=True, exist_ok=True)

    # Check if already downloaded
    extracted_dir = output_dir / dataset['extracted_dir']
    if extracted_dir.exists():
        print(f"Dataset already exists at: {extracted_dir}")
        response = input("Re-download? (y/N): ").strip().lower()
        if response != 'y':
            return True

    # Download
    archive_path = output_dir / dataset['filename']
    if not archive_path.exists():
        success = download_with_progress(
            dataset['url'],
            archive_path,
            dataset['size_mb']
        )
        if not success:
            return False
    else:
        print(f"Archive already exists: {archive_path}")

    # Extract
    print()
    success = extract_archive(archive_path, output_dir)
    if not success:
        return False

    # Cleanup archive
    if not keep_archive:
        print(f"Removing archive: {archive_path}")
        archive_path.unlink()

    print()
    print(f"Dataset ready at: {extracted_dir}")
    return True


def convert_to_rosbag(dataset_dir: Path, output_bag: Path):
    """Convert EuRoC/TUM-VI format to ROS2 bag (placeholder)."""
    print(f"Converting {dataset_dir} to ROS2 bag...")
    print("Note: Full conversion requires ROS2 environment")
    print(f"Output would be: {output_bag}")
    # In a full implementation, this would:
    # 1. Read images from mav0/cam0/data/
    # 2. Read IMU from mav0/imu0/data.csv
    # 3. Write to ROS2 bag format
    print("Conversion script placeholder - run inside Docker container")


def main():
    parser = argparse.ArgumentParser(
        description='Download VIO test datasets',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Available datasets:
  euroc_mh01  - EuRoC MAV MH_01_easy (~1.6GB)
  euroc_v101  - EuRoC MAV V1_01_easy (~800MB)
  tumvi_room1 - TUM-VI room1 (~2.5GB)
  all         - Download all datasets

Examples:
  python scripts/download_test_datasets.py euroc_mh01
  python scripts/download_test_datasets.py all --keep-archives
        """
    )
    parser.add_argument(
        'dataset',
        type=str,
        help='Dataset to download (euroc_mh01, tumvi_room1, all)'
    )
    parser.add_argument(
        '--output-dir',
        type=str,
        default='test_data',
        help='Output directory (default: test_data)'
    )
    parser.add_argument(
        '--keep-archives',
        action='store_true',
        help='Keep downloaded archives after extraction'
    )
    parser.add_argument(
        '--list',
        action='store_true',
        help='List available datasets'
    )

    args = parser.parse_args()

    if args.list:
        print("Available datasets:")
        for dataset_id, info in DATASETS.items():
            print(f"  {dataset_id:15} - {info['name']} (~{info['size_mb']}MB)")
            print(f"                    {info['description']}")
        return

    output_dir = Path(args.output_dir)

    if args.dataset == 'all':
        print("Downloading all datasets...")
        print()
        for dataset_id in DATASETS.keys():
            success = download_dataset(dataset_id, output_dir, args.keep_archives)
            if not success:
                print(f"Failed to download: {dataset_id}")
            print()
    else:
        success = download_dataset(args.dataset, output_dir, args.keep_archives)
        if not success:
            sys.exit(1)

    print()
    print("=" * 60)
    print("Download complete!")
    print("=" * 60)
    print()
    print("Next steps:")
    print("1. Start the development container:")
    print("   docker compose -f docker-compose.dev.yml run -it dev_shell bash")
    print()
    print("2. Convert to ROS2 bag (if needed):")
    print("   ros2 bag convert -i test_data/MH_01_easy -o test_data/mh01.db3")
    print()
    print("3. Play the bag file:")
    print("   ros2 bag play test_data/mh01.db3")


if __name__ == '__main__':
    main()
