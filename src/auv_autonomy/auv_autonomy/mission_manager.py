#!/usr/bin/env python3

import rclpy

from rclpy.node import Node

from std_msgs.msg import Float32

from auv_interfaces.msg import Mission


class MissionManager(Node):

    def __init__(self):

        super().__init__('mission_manager')

        self.heading_pub = self.create_publisher(
            Float32,
            '/auv/desired_heading',
            10
        )

        self.depth_pub = self.create_publisher(
            Float32,
            '/auv/desired_depth',
            10
        )

        self.distance_pub = self.create_publisher(
            Float32,
            '/auv/desired_distance',
            10
        )

        self.create_subscription(
            Mission,
            '/auv/mission',
            self.mission_callback,
            10
        )

        self.get_logger().info(
            'Mission Manager Started'
        )

    def mission_callback(self, msg):

        heading = Float32()
        heading.data = msg.heading

        depth = Float32()
        depth.data = msg.depth

        distance = Float32()
        distance.data = msg.distance

        self.heading_pub.publish(heading)
        self.depth_pub.publish(depth)
        self.distance_pub.publish(distance)

        self.get_logger().info(
            f'MISSION RECEIVED | '
            f'Heading={msg.heading:.1f} '
            f'Depth={msg.depth:.1f} '
            f'Distance={msg.distance:.1f} '
            f'Speed={msg.inspection_speed:.2f} '
            f'Duration={msg.duration:.1f}'
        )



def main(args=None):

    rclpy.init(args=args)

    node = MissionManager()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
