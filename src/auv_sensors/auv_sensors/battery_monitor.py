#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import HistoryPolicy

from sensor_msgs.msg import BatteryState

from auv_interfaces.msg import Battery


class BatteryMonitor(Node):

    def __init__(self):
        super().__init__('battery_monitor')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.battery_pub = self.create_publisher(
            Battery,
            '/auv/battery',
            10
        )

        self.subscription = self.create_subscription(
            BatteryState,
            '/mavros/battery',
            self.battery_callback,
            qos
        )

        self.get_logger().info('Battery Monitor Node Started')

    def battery_callback(self, msg):

        battery_msg = Battery()

        battery_msg.voltage = msg.voltage
        battery_msg.current = msg.current
        battery_msg.percentage = msg.percentage * 100.0

        self.battery_pub.publish(battery_msg)

        self.get_logger().info(
            f'V={msg.voltage:.2f}V '
            f'I={msg.current:.2f}A '
            f'SOC={msg.percentage*100:.1f}%'
        )


def main(args=None):

    rclpy.init(args=args)

    node = BatteryMonitor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
