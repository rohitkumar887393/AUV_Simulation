import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy

import socket
import json


class JoyReceiver(Node):

    def __init__(self):
        super().__init__('joy_receiver')

        self.pub = self.create_publisher(
            Joy,
            '/joy',
            1
        )

        self.sock = socket.socket(
            socket.AF_INET,
            socket.SOCK_DGRAM
        )
        self.sock.setblocking(False)

        try:
            self.sock.bind(
                ('0.0.0.0', 5005)
            )
        except Exception as e:
            self.get_logger().warn(f'UDP socket bind warning: {e}')

        self.timer = self.create_timer(
            0.02,
            self.receive
        )

    def receive(self):
        latest_msg = None
        while True:
            try:
                data, _ = self.sock.recvfrom(4096)
                data = json.loads(data.decode())
                msg = Joy()
                msg.axes = [float(x) for x in data["axes"]]
                msg.buttons = [int(x) for x in data["buttons"]]
                latest_msg = msg
            except Exception:
                break

        if latest_msg is not None:
            self.pub.publish(latest_msg)


def main(args=None):

    rclpy.init(args=args)

    node = JoyReceiver()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
