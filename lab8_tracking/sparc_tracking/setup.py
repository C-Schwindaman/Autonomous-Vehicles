from setuptools import setup
from glob import glob
import os

package_name = 'sparc_tracking'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Pragyan Dahal',
    maintainer_email='dahalpr1@msu.edu',
    description='Point cloud-driven multi-sensor tracking node with RViz visualization and launch files.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tracking_node_skeleton = sparc_tracking.tracking_node_skeleton:main',
            'tracking_node_skeleton_plus_projection = sparc_tracking.tracking_node_skeleton_plus_projection:main',
        ],
    },
)
