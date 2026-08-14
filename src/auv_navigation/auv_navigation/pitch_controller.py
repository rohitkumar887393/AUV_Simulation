#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from auv_interfaces.msg import Orientation
from auv_navigation.pid import PID


class PitchController(Node):

    def __init__(self):
        super().__init__('pitch_controller')

        # Declare ROS2 parameters (allows on-the-fly tuning)
        self.declare_parameter('kp', 1.0)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.1)
        self.declare_parameter('dt', 0.05)                 # Timer rate: 20 Hz (0.05s)
        self.declare_parameter('integral_limit', 0.5)
        self.declare_parameter('output_limit', 1.0)         # Max elevator deflection normalized command
        self.declare_parameter('desired_pitch_deg', 0.0)    # Manual pitch setpoint override in degrees

        # Initialize PID controller using parameters
        self.pid = PID(
            kp=self.get_parameter('kp').value,
            ki=self.get_parameter('ki').value,
            kd=self.get_parameter('kd').value,
            dt=self.get_parameter('dt').value,
            integral_limit=self.get_parameter('integral_limit').value,
            output_limit=self.get_parameter('output_limit').value
        )

        # Subscribers
        self.create_subscription(
            Orientation,
            '/auv/orientation',
            self.orientation_callback,
            10
        )

        self.create_subscription(
            Float64,
            '/auv/desired_pitch',
            self.desired_pitch_callback,
            10
        )

        # Publisher
        self.elevator_pub = self.create_publisher(
            Float64,
            '/auv/elevator_cmd',
            10
        )

        # Variables
        self.current_pitch = 0.0
        self.target_pitch = 0.0
        self.desired_pitch_msg_received = False

        # Add parameter event callback to update PID gains in real-time
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Control Loop Timer (20 Hz)
        self.timer = self.create_timer(
            self.get_parameter('dt').value,
            self.control_loop
        )

        self.get_logger().info('Pitch Controller Node Initialized (20 Hz)')

    def parameter_callback(self, params):
        # Dynamically update PID gains and limits when changed via 'ros2 param set'
        for param in params:
            if param.name == 'kp':
                self.pid.kp = param.value
                self.get_logger().info(f"Updated Kp: {param.value}")
            elif param.name == 'ki':
                self.pid.ki = param.value
                self.get_logger().info(f"Updated Ki: {param.value}")
            elif param.name == 'kd':
                self.pid.kd = param.value
                self.get_logger().info(f"Updated Kd: {param.value}")
            elif param.name == 'integral_limit':
                self.pid.integral_limit = param.value
            elif param.name == 'output_limit':
                self.pid.output_limit = param.value
        
        # Accept parameter updates
        from rcl_interfaces.msg import SetParametersResult
        return SetParametersResult(successful=True)

    def orientation_callback(self, msg):
        # Orientation is assumed to be in radians.
        self.current_pitch = msg.pitch

    def desired_pitch_callback(self, msg):
        # Desired pitch from depth controller is in radians
        self.target_pitch = msg.data
        self.desired_pitch_msg_received = True

    def control_loop(self):
        # Determine the target pitch (radians)
        if not self.desired_pitch_msg_received:
            # If no outer depth loop is publishing, use the manual parameter override (converted to rad)
            manual_deg = self.get_parameter('desired_pitch_deg').value
            self.target_pitch = math.radians(manual_deg)

        # Run PID update
        elevator_out = self.pid.update(self.target_pitch, self.current_pitch)

        # Publish the command
        cmd_msg = Float64()
        cmd_msg.data = float(elevator_out)
        self.elevator_pub.publish(cmd_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PitchController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
