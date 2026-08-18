import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    config_file = os.path.join(
        get_package_share_directory('auv_simulation'),
        'config',
        'ideal_auv.yaml'
    )

    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='true',
        description='Enable simulation mode for AUV control stack'
    )

    enable_gui_arg = DeclareLaunchArgument(
        'enable_gui',
        default_value='true',
        description='Enable live 2D visualizer dashboard'
    )

    enable_3d_arg = DeclareLaunchArgument(
        'enable_3d',
        default_value='true',
        description='Enable standalone 3D visualizer viewer'
    )

    auv_control_node = Node(
        package='auv_teleop',
        executable='auv_control',
        name='auv_control',
        output='screen',
        parameters=[{
            'use_sim': LaunchConfiguration('use_sim')
        }]
    )

    ideal_auv_node = Node(
        package='auv_simulation',
        executable='ideal_auv',
        name='ideal_auv',
        output='screen',
        parameters=[config_file]
    )

    sensor_simulator_node = Node(
        package='auv_simulation',
        executable='sensor_simulator',
        name='sensor_simulator',
        output='screen'
    )

    web_visualizer_node = Node(
        package='auv_simulation',
        executable='web_visualizer',
        name='web_visualizer',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_3d'))
    )

    odometry_node = Node(
        package='auv_navigation',
        executable='odometry_node',
        name='odometry_node',
        output='screen'
    )

    roll_stabilizer_node = Node(
        package='auv_autonomy',
        executable='roll_stabilizer',
        name='roll_stabilizer',
        output='screen'
    )

    heading_hold_node = Node(
        package='auv_autonomy',
        executable='heading_hold',
        name='heading_hold',
        output='screen'
    )

    depth_hold_node = Node(
        package='auv_autonomy',
        executable='depth_hold',
        name='depth_hold',
        output='screen'
    )

    distance_hold_node = Node(
        package='auv_autonomy',
        executable='distance_hold',
        name='distance_hold',
        output='screen'
    )

    mission_manager_node = Node(
        package='auv_autonomy',
        executable='mission_manager',
        name='mission_manager',
        output='screen'
    )

    inspection_mission_node = Node(
        package='auv_autonomy',
        executable='inspection_mission',
        name='inspection_mission',
        output='screen'
    )

    return LaunchDescription([
        use_sim_arg,
        enable_gui_arg,
        enable_3d_arg,
        auv_control_node,
        ideal_auv_node,
        sensor_simulator_node,
        web_visualizer_node,
        odometry_node,
        roll_stabilizer_node,
        heading_hold_node,
        depth_hold_node,
        distance_hold_node,
        mission_manager_node,
        inspection_mission_node
    ])

