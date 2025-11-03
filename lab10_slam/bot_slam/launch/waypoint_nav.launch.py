import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('bot_slam')
    waypoint_file_path = os.path.join(pkg_share, 'config', 'waypoints.json')
    waypoint_follower_node = Node(
        package='bot_slam',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[
            {'waypoint_file_path': waypoint_file_path}
        ]
    )
    return LaunchDescription([
        waypoint_follower_node
    ])