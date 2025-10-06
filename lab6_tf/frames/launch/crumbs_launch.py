import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    gazebo_pkg = get_package_share_directory('turtlebot3_gazebo')
    bringup_pkg = get_package_share_directory('turtlebot3_bringup')
    
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg, 'launch', 'empty_world.launch.py')
        )
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'rviz2.launch.py')
        )
    )

    crumbs_node = Node(
        package='frames',
        executable='crumbs',
        name='crumbs_node'
    )

    return LaunchDescription([
        gazebo_launch,
        rviz_launch,
        crumbs_node
    ])