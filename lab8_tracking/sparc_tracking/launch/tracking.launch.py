from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_name = 'sparc_tracking'

    package_share = get_package_share_directory(package_name)

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(package_share, 'config', 'tracking_params.yaml'),
        description='Path to the parameter file to use.'
    )

    play_bag_arg = DeclareLaunchArgument(
        'play_bag',
        default_value='true',
        description='Whether to play a rosbag automatically.'
    )

    bag_path_arg = DeclareLaunchArgument(
        'bag_path',
        default_value=os.path.expanduser('~/av/data/iac_bags'),
        description='Directory containing the rosbag to play.'
    )

    tracking_node = Node(
        package=package_name,
        executable='tracking_node_skeleton',
        name='sparc_style_tracker',
        output='screen',
        parameters=[LaunchConfiguration('params_file')]
    )

    def launch_setup(context, *args, **kwargs):
        actions = [tracking_node]
        if LaunchConfiguration('play_bag').perform(context).lower() in ('true', '1', 'yes'):
            bag_path = os.path.expanduser(LaunchConfiguration('bag_path').perform(context))
            if not bag_path:
                raise RuntimeError('bag_path is empty while play_bag is enabled.')
            if not os.path.exists(bag_path):
                raise FileNotFoundError(f'Rosbag path does not exist: {bag_path}')
            actions.append(
                ExecuteProcess(
                    cmd=['ros2', 'bag', 'play', bag_path],
                    output='screen'
                )
            )
        return actions

    launch_actions = [
        params_file_arg,
        play_bag_arg,
        bag_path_arg,
        OpaqueFunction(function=launch_setup)
    ]

    return LaunchDescription(launch_actions)
