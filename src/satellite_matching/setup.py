from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'satellite_matching'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Proxigo Team',
    maintainer_email='dev@proxigo.io',
    description='Satellite Matching - Aerial-to-satellite image matching',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'satellite_matcher_node = satellite_matching.satellite_matcher_node:main',
            'region_manager_node = satellite_matching.region_manager_node:main',
        ],
    },
)
