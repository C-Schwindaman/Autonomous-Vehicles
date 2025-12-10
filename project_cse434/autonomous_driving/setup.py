from setuptools import setup
import os
from glob import glob

package_name = 'autonomous_driving'

setup(
    name=package_name,
    version='0.0.0',
    packages=[
        package_name,
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='schwind7',
    maintainer_email='schwind7@msu.edu',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lane_detector_node = autonomous_driving.lane_detector_node:main',
            'collect_signs = autonomous_driving.collect_signs:main',
            'sign_detector = autonomous_driving.sign_detector:main',
            'obstacle_detector_node = autonomous_driving.obstacle_detector_node:main',
            'decision_maker = autonomous_driving.decision_maker:main',
        ],
    },
)