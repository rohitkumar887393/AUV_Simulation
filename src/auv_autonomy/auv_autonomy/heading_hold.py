#!/usr/bin/env python3

import math
import json
import rclpy

from rclpy.node import Node

from sensor_msgs.msg import Joy
from std_msgs.msg import Float32
from std_msgs.msg import String

from auv_interfaces.msg import Orientation

from .pid import PID


class HeadingHold(Node):

    def __init__(self):

        super().__init__('heading_hold')

        self.declare_parameter('kp', 6.0)
        self.declare_parameter('ki', 0.05)
        self.declare_parameter('kd', 1.5)

        kp = self.get_parameter('kp').value
        ki = self.get_parameter('ki').value
        kd = self.get_parameter('kd').value

        self.pid = PID(
            kp,
            ki,
            kd,
            -300.0,
            300.0
        )

        self.current_heading = 0.0
        self.target_heading = 0.0
        self.heading_error = 0.0

        self.heading_hold_enabled = False

        self.prev_lb = 0
        self.prev_rb = 0

        self.prev_time = self.get_clock().now()

        self.create_subscription(
            Orientation,
            '/auv/orientation',
            self.orientation_callback,
            10
        )

        self.create_subscription(
            Float32,
            '/auv/desired_heading',
            self.desired_heading_callback,
            10
        )

        self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

        self.rudder_pub = self.create_publisher(
            Float32,
            '/auv/rudder_cmd',
            10
        )

        self.status_pub = self.create_publisher(
            String,
            '/auv/heading_hold_status',
            10
        )

        self.state_pub = self.create_publisher(
            Float32,
            '/auv/heading_hold_enabled',
            10
        )

        self.telemetry_pub = self.create_publisher(
            String,
            '/auv/heading_telemetry',
            10
        )

        self.get_logger().info(
            'Heading Hold Started (Continuous Closed-Loop Control Active)'
        )

        self.target_pub = self.create_publisher(
            Float32,
            '/auv/desired_heading',
            10
        )

    def desired_heading_callback(self, msg):
        self.target_heading = float(msg.data)
        self.heading_hold_enabled = True

        self.pid.integral = 0.0
        self.pid.prev_error = 0.0
        self.prev_time = self.get_clock().now()

        state = Float32()
        state.data = 1.0
        self.state_pub.publish(state)

        status = String()
        status.data = f"ON:{self.target_heading:.1f}"
        self.status_pub.publish(status)

        self.get_logger().info(f"DESIRED HEADING SET : {self.target_heading:.1f}°")

    def joy_callback(self, msg):

        lb_btn = msg.buttons[4]   # LB
        rb_btn = msg.buttons[5]   # RB

        # RB -> Enable Heading Hold
        if rb_btn == 1 and self.prev_rb == 0:
            self.target_heading = self.current_heading

            target = Float32()
            target.data = self.target_heading
            self.target_pub.publish(target)

            self.pid.integral = 0.0
            self.pid.prev_error = 0.0
            self.prev_time = self.get_clock().now()

            self.heading_hold_enabled = True

            state = Float32()
            state.data = 1.0
            self.state_pub.publish(state)

            status = String()
            status.data = f"ON:{self.target_heading:.1f}"
            self.status_pub.publish(status)

            self.get_logger().info(
                f"HEADING HOLD ON : {self.target_heading:.1f}°"
            )

        # LB -> Disable Heading Hold
        if lb_btn == 1 and self.prev_lb == 0:
            self.heading_hold_enabled = False

            state = Float32()
            state.data = 0.0
            self.state_pub.publish(state)

            status = String()
            status.data = "OFF"
            self.status_pub.publish(status)

            cmd = Float32()
            cmd.data = 0.0
            self.rudder_pub.publish(cmd)

            self.get_logger().info(
                "HEADING HOLD OFF"
            )

        self.prev_lb = lb_btn
        self.prev_rb = rb_btn

    def orientation_callback(self, msg):
        self.current_heading = float(msg.yaw)

        if not self.heading_hold_enabled:
            return

        # Compute wrapped error [-180, +180]
        error = self.target_heading - self.current_heading
        while error > 180.0:
            error -= 360.0
        while error < -180.0:
            error += 360.0

        self.heading_error = error

        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds / 1e9

        if dt <= 0.0:
            return

        # Anti-windup integral clamping
        if abs(error) > 20.0:
            self.pid.integral = 0.0
        else:
            self.pid.integral = max(-30.0, min(30.0, self.pid.integral + error * dt))

        # PID Update
        output = self.pid.update(error, dt)

        cmd = Float32()
        cmd.data = float(output)
        self.rudder_pub.publish(cmd)

        self.prev_time = now

        # Publish Heading Debug Telemetry (Requirement 12)
        telem_dict = {
            "target": self.target_heading,
            "current": self.current_heading,
            "error": error,
            "output": cmd.data
        }
        telem_msg = String()
        telem_msg.data = json.dumps(telem_dict)
        self.telemetry_pub.publish(telem_msg)

        # Logging (Requirement 3)
        self.get_logger().info(
            f"HDG: Target={self.target_heading:.1f}° "
            f"Current={self.current_heading:.1f}° "
            f"Error={error:.1f}° "
            f"PID_Out={cmd.data:.1f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = HeadingHold()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
