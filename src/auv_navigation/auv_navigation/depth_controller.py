#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from auv_interfaces.msg import Depth
from auv_navigation.pid import PID


class DepthController(Node):

    def __init__(self):
        super().__init__('depth_controller')

        # Declare ROS2 parameters
        self.declare_parameter('kp', 80)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.0)


        now = self.get_clock().now()

        dt = (
            now - self.prev_time
        ).nanoseconds / 1e9

        self.prev_time = now


        self.declare_parameter('output_limit_deg', 40.0)     # Max pitch command in degrees (e.g. ±25 deg)

        # Read parameters
        kp = self.get_parameter('kp').value
        ki = self.get_parameter('ki').value
        kd = self.get_parameter('kd').value
        dt = self.get_parameter('dt').value
        integral_limit = self.get_parameter('integral_limit').value






        
        # Output limit is the maximum pitch command angle (converted to radians)
        max_pitch_deg = self.get_parameter('output_limit_deg').value
        output_limit_rad = math.radians(max_pitch_deg)

        # Initialize PID controller
        self.pid = PID(
            kp=kp,
            ki=ki,
            kd=kd,
            dt=dt,
            integral_limit=integral_limit,
            output_limit=output_limit_rad
        )

        # Subscribers
        self.create_subscription(
            Depth,
            '/auv/depth',
            self.depth_callback,
            10
        )

        # Publisher
        self.pitch_pub = self.create_publisher(
            Float64,
            '/auv/desired_pitch',
            10
        )

        # Variables
        self.current_depth = 0.0

        # Parameter update callback
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Control Loop Timer (10 Hz)
        self.timer = self.create_timer(
            dt,
            self.control_loop
        )

        self.get_logger().info('Depth Controller Node Initialized (10 Hz)')

    def parameter_callback(self, params):
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
            elif param.name == 'output_limit_deg':
                self.pid.output_limit = math.radians(param.value)
                self.get_logger().info(f"Updated Output Limit (rad): {self.pid.output_limit}")
        
        from rcl_interfaces.msg import SetParametersResult
        return SetParametersResult(successful=True)

    def depth_callback(self, msg):
        self.current_depth = msg.depth

    def control_loop(self):
        target_depth = self.get_parameter('desired_depth_m').value

        # Calculate standard PID output based on depth error
        # Note: Setpoint = target_depth, Measurement = current_depth
        pid_output = self.pid.update(target_depth, self.current_depth)

        # Sign Correction:
        # If target_depth > current_depth (need to go deeper, error is positive):
        # To go down, the torpedo nose must pitch DOWN (negative pitch).
        # Therefore, we invert the PID output to match physical convention.
        desired_pitch_rad = -pid_output

        # Publish target pitch
        msg = Float64()
        msg.data = float(desired_pitch_rad)
        self.pitch_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DepthController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
