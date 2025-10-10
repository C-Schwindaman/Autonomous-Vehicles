from setuptools import find_packages, setup

package_name = 'yolo_det'

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
    description='A ROS 2 package that provides a node to sample and save images from a camera feed for computer vision training',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sample_img = yolo_det.sample_img:main',
            'det_img = yolo_det.det_img:main',
        ],
    },
)
