from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        Node(
            package='auv_autonomy',
            executable='roll_stabilizer',
            name='roll_stabilizer',
            output='screen'
        ),

        Node(
            package='auv_autonomy',
            executable='heading_hold',
            name='heading_hold',
            output='screen'
        ),

        Node(
            package='auv_autonomy',
            executable='depth_hold',
            name='depth_hold',
            output='screen'
        ),

        Node(
            package='auv_autonomy',
            executable='distance_hold',
            name='distance_hold',
            output='screen'
        ),

        Node(
            package='auv_autonomy',
            executable='mission_manager',
            name='mission_manager',
            output='screen'
        ),

        Node(
            package='auv_autonomy',
            executable='inspection_mission',
            name='inspection_mission',
            output='screen'
        )
    ])

