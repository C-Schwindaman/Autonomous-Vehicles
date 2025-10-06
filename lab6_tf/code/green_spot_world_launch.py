import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    curvy_road_pkg = get_package_share_directory('curvy_road')
    frames_pkg = get_package_share_directory('frames')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(curvy_road_pkg, 'launch', 'curvy_road.launch.py')
            ),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(frames_pkg, 'launch', 'green_spot_launch.py')
            ),
        ),

    ])
