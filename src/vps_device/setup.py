from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'vps_device'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools', 'numpy', 'opencv-python', 'scikit-learn'],
    zip_safe=True,
    maintainer='Proxigo Team',
    maintainer_email='dev@proxigo.io',
    description='VPS Device - Visual Positioning System device module (standalone + ROS2)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vps_device_node = vps_device.vps_device_node:main',
        ],
    },
)
