#!/usr/bin/env python3

import math
import json
import rclpy

from rclpy.node import Node

from std_msgs.msg import Float32
from std_msgs.msg import String
from auv_interfaces.msg import Odometry

from .pid import PID


class DistanceHold(Node):

    def __init__(self):

        super().__init__('distance_hold')

        self.current_x = 0.0
        self.current_y = 0.0

        self.start_x = 0.0
        self.start_y = 0.0

        self.target_distance = 0.0
        self.travelled_distance = 0.0
        self.remaining_distance = 0.0

        self.mission_active = False

        self.declare_parameter('kp', 0.15)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.0)

        kp = self.get_parameter('kp').value
        ki = self.get_parameter('ki').value
        kd = self.get_parameter('kd').value

        self.pid = PID(
            kp,
            ki,
            kd,
            0.0,
            1.0
        )

        self.prev_time = self.get_clock().now()

        # Position subscription from DVL sensor simulator / Odometry
        self.create_subscription(
            Odometry,
            '/auv/dvl/position',
            self.position_callback,
            10
        )

        # Autonomy target distance subscription
        self.create_subscription(
            Float32,
            '/auv/desired_distance',
            self.distance_callback,
            10
        )

        # Publishers
        self.throttle_pub = self.create_publisher(
            Float32,
            '/auv/throttle_cmd',
            10
        )

        self.state_pub = self.create_publisher(
            Float32,
            '/auv/distance_hold_enabled',
            10
        )

        self.status_pub = self.create_publisher(
            String,
            '/auv/mission_status',
            10
        )

        self.dist_status_pub = self.create_publisher(
            String,
            '/auv/distance_hold_status',
            10
        )

        self.travelled_pub = self.create_publisher(
            Float32,
            '/auv/distance_travelled',
            10
        )

        self.remaining_pub = self.create_publisher(
            Float32,
            '/auv/distance_remaining',
            10
        )

        self.telemetry_pub = self.create_publisher(
            String,
            '/auv/distance_telemetry',
            10
        )

        self.get_logger().info(
            'Distance Hold Started (Feedback Bug Fix Applied)'
        )

        status = String()
        status.data = "IDLE"
        self.status_pub.publish(status)

        dist_stat = String()
        dist_stat.data = "OFF"
        self.dist_status_pub.publish(dist_stat)

    def distance_callback(self, msg):
        new_target = float(msg.data)

        if new_target <= 0.0:
            self.mission_active = False
            self.target_distance = 0.0
            self.travelled_distance = 0.0
            self.remaining_distance = 0.0
            self.pid.integral = 0.0
            self.pid.prev_error = 0.0

            cmd = Float32()
            cmd.data = 0.0
            self.throttle_pub.publish(cmd)

            state = Float32()
            state.data = 0.0
            self.state_pub.publish(state)

            dist_stat = String()
            dist_stat.data = "OFF"
            self.dist_status_pub.publish(dist_stat)

            self.get_logger().info('Distance Target Cleared (0.0 m)')
            return

        # Start new distance mission — capture current vehicle position
        self.target_distance = new_target
        self.start_x = self.current_x
        self.start_y = self.current_y

        self.travelled_distance = 0.0
        self.remaining_distance = self.target_distance
        self.mission_active = True

        self.pid.integral = 0.0
        self.pid.prev_error = 0.0
        self.prev_time = self.get_clock().now()

        status = String()
        status.data = "RUNNING"
        self.status_pub.publish(status)

        dist_stat = String()
        dist_stat.data = f"ON:{self.target_distance:.1f}"
        self.dist_status_pub.publish(dist_stat)

        state = Float32()
        state.data = 1.0
        self.state_pub.publish(state)

        self.get_logger().info(
            f'Start Distance Mission: Target={self.target_distance:.2f} m | Start Pose=({self.start_x:.2f}, {self.start_y:.2f})'
        )

    def position_callback(self, msg):
        self.current_x = float(msg.x)
        self.current_y = float(msg.y)

        if not self.mission_active:
            # Publish idle distance telemetry
            trav_msg = Float32()
            trav_msg.data = float(self.travelled_distance)
            self.travelled_pub.publish(trav_msg)

            rem_msg = Float32()
            rem_msg.data = float(self.remaining_distance)
            self.remaining_pub.publish(rem_msg)
            return

        # Compute continuous planar vehicle displacement from mission start position
        dx = self.current_x - self.start_x
        dy = self.current_y - self.start_y

        self.travelled_distance = math.sqrt(dx * dx + dy * dy)
        self.remaining_distance = max(0.0, self.target_distance - self.travelled_distance)
        distance_error = self.target_distance - self.travelled_distance

        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds / 1e9
        self.prev_time = now

        # Update Distance PID
        output = self.pid.update(self.remaining_distance, dt)

        cmd = Float32()
        cmd.data = float(output)

        # Distance Completion Logic (Tolerance <= 0.45 m)
        if self.remaining_distance <= 0.45:
            cmd.data = 0.0

            state = Float32()
            state.data = 0.0
            self.state_pub.publish(state)

            status = String()
            status.data = "COMPLETE"
            self.status_pub.publish(status)

            dist_stat = String()
            dist_stat.data = "OFF"
            self.dist_status_pub.publish(dist_stat)

            self.mission_active = False

            self.get_logger().info(
                f'Distance Target Reached: Travelled={self.travelled_distance:.2f} m / Target={self.target_distance:.2f} m'
            )

        self.throttle_pub.publish(cmd)

        # Publish Live Telemetry
        trav_msg = Float32()
        trav_msg.data = float(self.travelled_distance)
        self.travelled_pub.publish(trav_msg)

        rem_msg = Float32()
        rem_msg.data = float(self.remaining_distance)
        self.remaining_pub.publish(rem_msg)

        telem_data = {
            "target": self.target_distance,
            "travelled": self.travelled_distance,
            "remaining": self.remaining_distance,
            "error": distance_error,
            "output": cmd.data,
            "start_x": self.start_x,
            "start_y": self.start_y,
            "current_x": self.current_x,
            "current_y": self.current_y
        }
        telem_msg = String()
        telem_msg.data = json.dumps(telem_data)
        self.telemetry_pub.publish(telem_msg)

        # Required ROS Debug Logging (Requirement 11)
        self.get_logger().info(
            f"DIST: X={self.current_x:.2f} Y={self.current_y:.2f} "
            f"START_X={self.start_x:.2f} START_Y={self.start_y:.2f} "
            f"TRAVELLED={self.travelled_distance:.2f}m TARGET={self.target_distance:.2f}m "
            f"REMAINING={self.remaining_distance:.2f}m ERROR={distance_error:.2f}m PID={cmd.data:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = DistanceHold()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
