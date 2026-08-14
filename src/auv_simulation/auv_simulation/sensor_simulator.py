#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, String
from geometry_msgs.msg import TwistStamped

from auv_interfaces.msg import (
    Orientation,
    Depth,
    Odometry
)


class SensorSimulator(Node):

    def __init__(self):
        super().__init__('sensor_simulator')

        # Subscriptions
        self.create_subscription(
            Odometry,
            '/auv/ideal_state',
            self.state_callback,
            1
        )

        # Publishers
        self.orientation_pub = self.create_publisher(
            Orientation,
            '/auv/orientation',
            1
        )

        self.depth_pub = self.create_publisher(
            Depth,
            '/auv/depth',
            1
        )

        self.dvl_vel_pub = self.create_publisher(
            TwistStamped,
            '/auv/dvl/velocity',
            1
        )

        self.dvl_pos_pub = self.create_publisher(
            Odometry,
            '/auv/dvl/position',
            1
        )

        self.dvl_alt_pub = self.create_publisher(
            Float32,
            '/auv/dvl/altitude',
            10
        )

        self.dvl_status_pub = self.create_publisher(
            String,
            '/auv/dvl/status',
            10
        )

        self.get_logger().info('Sensor Telemetry Simulator Running (Ideal Noise-Free Mode)')

    def state_callback(self, msg):
        now_msg = self.get_clock().now().to_msg()

        # 1. Orientation (/auv/orientation)
        orient_msg = Orientation()
        orient_msg.roll = float(msg.roll)
        orient_msg.pitch = float(msg.pitch)
        orient_msg.yaw = float(msg.yaw)
        self.orientation_pub.publish(orient_msg)

        # 2. Depth (/auv/depth)
        depth_msg = Depth()
        depth_msg.depth = float(msg.z)
        # Pressure (mbar) = Surface pressure (~1013.25) + hydrostatic pressure (depth * 98.0665 mbar/m)
        depth_msg.pressure = float(1013.25 + msg.z * 98.0665)
        depth_msg.temperature = 20.0
        self.depth_pub.publish(depth_msg)

        # 3. DVL Velocity (/auv/dvl/velocity)
        twist_msg = TwistStamped()
        twist_msg.header.stamp = now_msg
        twist_msg.header.frame_id = 'dvl_link'
        twist_msg.twist.linear.x = float(msg.vx)
        twist_msg.twist.linear.y = float(msg.vy)
        twist_msg.twist.linear.z = float(msg.vz)
        self.dvl_vel_pub.publish(twist_msg)

        # 4. DVL Position (/auv/dvl/position)
        dvl_pos_msg = Odometry()
        dvl_pos_msg.x = float(msg.x)
        dvl_pos_msg.y = float(msg.y)
        dvl_pos_msg.z = float(msg.z)
        dvl_pos_msg.roll = float(msg.roll)
        dvl_pos_msg.pitch = float(msg.pitch)
        dvl_pos_msg.yaw = float(msg.yaw)
        dvl_pos_msg.vx = float(msg.vx)
        dvl_pos_msg.vy = float(msg.vy)
        dvl_pos_msg.vz = float(msg.vz)
        self.dvl_pos_pub.publish(dvl_pos_msg)

        # 5. DVL Altitude (/auv/dvl/altitude)
        alt_msg = Float32()
        alt_msg.data = max(0.0, float(10.0 - msg.z))
        self.dvl_alt_pub.publish(alt_msg)

        # 6. DVL Status (/auv/dvl/status)
        status_msg = String()
        status_msg.data = "VALID"
        self.dvl_status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SensorSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
