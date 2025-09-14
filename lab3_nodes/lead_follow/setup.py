from setuptools import find_packages, setup

package_name = 'lead_follow'

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
    description='A simple ROS2 package for a follow-the-leader robot simulation.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'leader = lead_follow.leader:main',
            'follower = lead_follow.follower:main',
            'leader_random = lead_follow.leader_random:main',
        ],
    },
)
