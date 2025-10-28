from setuptools import find_packages, setup

package_name = 'lane_follow'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Cameron Schwindaman',
    maintainer_email='schwind7@msu.edu',
    description='ROS 2 package for Lab 9: Lane Detection and Following',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detect = lane_follow.detect:main',
            'follow_pid = lane_follow.follow_pid:main',
            'ground_spot = lane_follow.ground_spot:main',
            'follow_pure_pursuit = lane_follow.follow_pure_pursuit:main',
        ],
    },
)
