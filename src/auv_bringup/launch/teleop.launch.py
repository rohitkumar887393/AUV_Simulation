from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim',
            default_value='false',
            description='Enable simulation mode for AUV control'
        ),

        Node(
            package='auv_teleop',
            executable='receiver',
            output='screen'
        ),

        Node(
            package='auv_teleop',
            executable='auv_control',
            output='screen',
            parameters=[{
                'use_sim': LaunchConfiguration('use_sim')
            }]
        ),

        Node(
            package='auv_autonomy',
            executable='depth_hold',
            output='screen'
        ),

        Node(
            package='auv_autonomy',
            executable='roll_stabilizer',
            output='screen'
        ),
    ])

