import os
import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Find the path to the package and specify the JSON file relative to this
    json_path = os.path.join(
        get_package_share_directory('frames'),
        'models',
        'logr_coefs.json'
    )

    # Launch the nodes
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='frames',
            executable='ground_spot',  
            name='locate_green_spot',  # override default node name
            arguments=['--json_path', json_path],
        )
    ])
