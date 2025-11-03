from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'bot_slam'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        (os.path.join('share', package_name, 'launch'), 
            glob(os.path.join('launch', '*.launch.py'))),
            
        (os.path.join('share', package_name, 'config'), 
            glob(os.path.join('config', '*.[jy][sa][om][nl]'))),
    
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Cameron Schwindaman',
    maintainer_email='schwind7@msu.edu',
    description='ROS 2 package for Lab 10: Mapping and obstacle clustering.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obstacles = bot_slam.obstacles:main',
            'waypoint_follower = bot_slam.waypoint_follower:main',
        ],
    },
)
