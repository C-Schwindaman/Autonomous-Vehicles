from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'frames'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.py')),
        (os.path.join('share', package_name, 'models'), glob('models/*.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Cameron Schwindaman',
    maintainer_email='schwind7@msu.edu',
    description="A ROS 2 package to drop TF frame crumbs along a robot's path",
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'crumbs = frames.crumbs:main',
            'ground_spot = frames.ground_spot:main',
        ],
    },
)
