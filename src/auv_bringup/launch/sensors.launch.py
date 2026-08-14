from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        Node(
            package='auv_sensors',
            executable='depth_sensor',
            output='screen'
        ),

        Node(
            package='auv_sensors',
            executable='imu_node',
            output='screen'
        ),

        Node(
            package='auv_sensors',
            executable='leak_sensor',
            output='screen'
        ),
 
        Node(
            package='auv_sensors',
            executable='battery_monitor',
            output='screen'
        ),
        Node(
            package='auv_sensors',
            executable='dvl_sensor',
            output='screen',
            parameters=[
                {
                    'sim_mode': False,
                    'dvl_ip': '192.168.194.95',
                    'dvl_port': 16171
                }
            ]
        ),


        Node(
            package='auv_navigation',
            executable='odometry_node',
            output='screen',
        ),

    ])
