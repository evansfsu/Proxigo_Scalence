from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'state_fusion'

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
    description='State Fusion - VIO + satellite matching fusion',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fusion_node = state_fusion.fusion_node:main',
            'mavros_bridge_node = state_fusion.mavros_bridge_node:main',
        ],
    },
)
