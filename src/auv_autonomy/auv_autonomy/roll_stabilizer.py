#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from auv_interfaces.msg import Orientation


# Inline self-contained PID class to avoid cross-package build dependencies
class PID:
    def __init__(self, kp=0.0, ki=0.0, kd=0.0, integral_limit=None, output_limit=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = integral_limit
        self.output_limit = output_limit

        self.integral = 0.0
        self.previous_error = 0.0
        self.first_run = True

    def update(self, setpoint, measurement, dt):
        if dt <= 0.0:
            return 0.0
            
        error = setpoint - measurement

        # Proportional term
        p_out = self.kp * error

        # Integral term
        self.integral += error * dt
        if self.integral_limit is not None:
            self.integral = max(min(self.integral, self.integral_limit), -self.integral_limit)
        i_out = self.ki * self.integral

        # Derivative term
        if self.first_run:
            d_out = 0.0
            self.first_run = False
        else:
            derivative = (error - self.previous_error) / dt
            d_out = self.kd * derivative

        self.previous_error = error

        # Combined output
        output = p_out + i_out + d_out

        # Output clamping
        if self.output_limit is not None:
            output = max(min(output, self.output_limit), -self.output_limit)

        return output
        

    def reset(self):
        self.integral = 0.0
        self.previous_error = 0.0
        self.first_run = True


class RollStabilizer(Node):

    def __init__(self):
        super().__init__('roll_stabilizer')

        # Declare ROS2 Parameters for real-time PID tuning
        self.declare_parameter('kp', 4.0)                # Default matching your original P gain
        self.declare_parameter('ki', 0.01)
        self.declare_parameter('kd', 0.2)
        self.declare_parameter('integral_limit', 100.0)
        self.declare_parameter('output_limit', 300.0)    # Matches the max/min clamp in old code

        # Initialize the PID controller
        self.pid = PID(
            kp=self.get_parameter('kp').value,
            ki=self.get_parameter('ki').value,
            kd=self.get_parameter('kd').value,
            integral_limit=self.get_parameter('integral_limit').value,
            output_limit=self.get_parameter('output_limit').value
        )

        self.roll = 0.0
        self.last_time = None
        self.debug_counter = 0


        # Subscribers
        self.create_subscription(
            Orientation,
            '/auv/orientation',
            self.orientation_callback,
            10
        )

        # Publisher
        self.roll_pub = self.create_publisher(
            Float32,
            '/auv/roll_cmd',
            10
        )

        # Add parameter event callback for live tuning
        self.add_on_set_parameters_callback(self.parameter_callback)


    def parameter_callback(self, params):
        for param in params:
            if param.name == 'kp':
                self.pid.kp = param.value
                self.get_logger().info(f"Updated Roll Kp: {param.value}")
            elif param.name == 'ki':
                self.pid.ki = param.value
                self.get_logger().info(f"Updated Roll Ki: {param.value}")
            elif param.name == 'kd':
                self.pid.kd = param.value
                self.get_logger().info(f"Updated Roll Kd: {param.value}")
            elif param.name == 'integral_limit':
                self.pid.integral_limit = param.value
            elif param.name == 'output_limit':
                self.pid.output_limit = param.value
                
        from rcl_interfaces.msg import SetParametersResult
        return SetParametersResult(successful=True)

    def orientation_callback(self, msg):
        self.roll = msg.roll
        
        # Calculate dynamic dt based on actual message arrival times
        now = self.get_clock().now()
        if self.last_time is None:
            self.last_time = now
            return
            
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        # Run PID update to keep roll at 0.0
        # Setpoint = 0.0, Measurement = self.roll
        roll_cmd = self.pid.update(0.0, self.roll, dt)

        # Publish the command
        cmd = Float32()
        cmd.data = float(roll_cmd)
        self.roll_pub.publish(cmd)

        self.get_logger().info(
            f'Roll={self.roll:.2f} deg | Cmd={roll_cmd:.1f} | dt={dt:.3f}s'
        )


def main(args=None):
    rclpy.init(args=args)
    node = RollStabilizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
