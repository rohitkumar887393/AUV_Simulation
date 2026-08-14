#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped

from auv_interfaces.msg import (
    Orientation,
    Depth,
    Odometry
)


class OdometryNode(Node):

    def __init__(self):

        super().__init__('odometry_node')

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        self.depth = 0.0

        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0

        self.x = 0.0
        self.y = 0.0

        self.last_time = self.get_clock().now()

        self.create_subscription(
            Orientation,
            '/auv/orientation',
            self.orientation_callback,
            10
        )

        self.create_subscription(
            Depth,
            '/auv/depth',
            self.depth_callback,
            10
        )

        self.create_subscription(
            TwistStamped,
            '/auv/dvl/velocity',
            self.dvl_callback,
            10
        )

        self.odom_pub = self.create_publisher(
            Odometry,
            '/auv/odometry',
            10
        )

        self.create_timer(
            0.1,
            self.publish_odometry
        )

        self.get_logger().info(
            'Odometry Node Started'
        )

    def orientation_callback(self, msg):
        self.roll = msg.roll
        self.pitch = msg.pitch
        self.yaw = msg.yaw

    def depth_callback(self, msg):
        self.depth = msg.depth

    def dvl_callback(self, msg):

        now = self.get_clock().now()

        dt = (
            now - self.last_time
        ).nanoseconds / 1e9

        self.last_time = now

        self.vx = msg.twist.linear.x
        self.vy = msg.twist.linear.y
        self.vz = msg.twist.linear.z

        yaw_rad = math.radians(self.yaw)

        vx_world = (
            self.vx * math.cos(yaw_rad)
            - self.vy * math.sin(yaw_rad)
        )

        vy_world = (
            self.vx * math.sin(yaw_rad)
            + self.vy * math.cos(yaw_rad)
        )

        self.x += vx_world * dt
        self.y += vy_world * dt

    def publish_odometry(self):

        msg = Odometry()

        msg.x = self.x
        msg.y = self.y
        msg.z = self.depth

        msg.roll = self.roll
        msg.pitch = self.pitch
        msg.yaw = self.yaw

        msg.vx = self.vx
        msg.vy = self.vy
        msg.vz = self.vz

        self.odom_pub.publish(msg)


def main(args=None):

    rclpy.init(args=args)

    node = OdometryNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
