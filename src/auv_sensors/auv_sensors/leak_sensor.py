#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import HistoryPolicy

from mavros_msgs.msg import Mavlink

from auv_interfaces.msg import Leak


LEAK_PAYLOAD_1 = 7296992941814664194
LEAK_PAYLOAD_2 = 110386907145588


class LeakSensor(Node):

    def __init__(self):
        super().__init__('leak_sensor')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.leak_pub = self.create_publisher(
            Leak,
            '/auv/leak',
            10
        )

        self.subscription = self.create_subscription(
            Mavlink,
            '/uas1/mavlink_source',
            self.mavlink_callback,
            qos
        )

        self.get_logger().info('Leak Sensor Node Started')

    def mavlink_callback(self, msg):

        if msg.msgid != 253:
            return

        if len(msg.payload64) < 2:
            return

        leak_detected = (
            msg.payload64[0] == LEAK_PAYLOAD_1
            and
            msg.payload64[1] == LEAK_PAYLOAD_2
        )

        leak_msg = Leak()
        leak_msg.leak_detected = leak_detected

        self.leak_pub.publish(leak_msg)

        if leak_detected:
            self.get_logger().warn('LEAK DETECTED!')


def main(args=None):

    rclpy.init(args=args)

    node = LeakSensor()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
