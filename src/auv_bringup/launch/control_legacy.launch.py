import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Locate the default control parameters YAML file
    config_file = os.path.join(
        get_package_share_directory('auv_bringup'),
        'config',
        'control_params.yaml'
    )

    # Pitch Hold Controller Node (Inner Loop)
    pitch_node = Node(
        package='auv_navigation',
        executable='pitch_controller',
        name='pitch_controller',
        output='screen',
        parameters=[config_file]
    )

    # Depth Hold Controller Node (Outer Loop)
    depth_node = Node(
        package='auv_navigation',
        executable='depth_controller',
        name='depth_controller',
        output='screen',
        parameters=[config_file]
    )

    return LaunchDescription([
        pitch_node,
        depth_node
    ])
