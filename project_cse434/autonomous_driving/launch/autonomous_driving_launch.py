#!/usr/bin/env python3

import launch
import launch.conditions
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    # Get package directory
    pkg_dir = get_package_share_directory('autonomous_driving')
    
    # Declare launch arguments
    weights_dir_arg = DeclareLaunchArgument(
        'weights_dir',
        default_value=os.path.expanduser('~/av/hyunjoey_av/AV_Project_22/runs/detect/train3/weights'),
        description='Directory containing YOLO weights (best.pt)'
    )
    
    use_sign_detector_arg = DeclareLaunchArgument(
        'use_sign_detector',
        default_value='true',
        description='Enable sign detection'
    )
    
    # Get launch configurations
    weights_dir = LaunchConfiguration('weights_dir')
    use_sign_detector = LaunchConfiguration('use_sign_detector')
    
    # Lane Detector Node
    lane_detector = Node(
        package='autonomous_driving',
        executable='lane_detector_node',
        name='lane_detector_node',
        output='screen',
        parameters=[{
            'kp': 0.6,
            'ki': 0.0,
            'kd': 0.05,
            'linear_velocity': 0.15,
            'debug_mode': False  
        }]
    )
    
    # Obstacle Detector Node
    obstacle_detector = Node(
        package='autonomous_driving',
        executable='obstacle_detector_node',
        name='obstacle_detector_node',
        output='screen'
    )
    
    # Sign Detector Node
    sign_detector = Node(
        package='autonomous_driving',
        executable='sign_detector',
        name='sign_detector',
        output='screen',
        arguments=[
            '--weights_dir', weights_dir,
            '--topic', '/camera/image_raw',
            '--conf', '0.5',
            '--rate', '5.0'
        ],
        condition=launch.conditions.IfCondition(use_sign_detector)
    )
    
    # Decision Maker Node
    decision_maker = Node(
        package='autonomous_driving',
        executable='decision_maker',
        name='decision_maker',
        output='screen',
        parameters=[{
            'obstacle_stop_distance': 0.5,
            'obstacle_warning_distance': 1.0,
            'stop_sign_duration': 3.0,
            'linear_velocity': 0.2
        }]
    )
    
    return LaunchDescription([
        weights_dir_arg,
        use_sign_detector_arg,
        lane_detector,
        obstacle_detector,
        sign_detector,
        decision_maker
    ])