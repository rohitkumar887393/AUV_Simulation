#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from auv_interfaces.msg import ActuatorCmds


class ActuatorSimulator(Node):

    def __init__(self):
        super().__init__('actuator_simulator')

        self.create_subscription(
            ActuatorCmds,
            '/auv/actuator_cmds',
            self.actuator_callback,
            10
        )

        self.get_logger().info('Actuator Simulator Node Initialized (Phase 1)')

    def actuator_callback(self, msg):
        self.get_logger().info(
            f"Actuator Cmds Received | "
            f"Elevator L: {msg.elevator_left} us | "
            f"Elevator R: {msg.elevator_right} us | "
            f"Rudder L: {msg.rudder_left} us | "
            f"Rudder R: {msg.rudder_right} us | "
            f"Thruster: {msg.main_thruster} us"
        )


def main(args=None):
    rclpy.init(args=args)
    node = ActuatorSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
