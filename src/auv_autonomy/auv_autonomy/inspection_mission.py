#!/usr/bin/env python3

import math
import json
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, String, Bool
from geometry_msgs.msg import TwistStamped
from auv_interfaces.msg import Depth, Orientation, Odometry, Mission

from .pid import PID


class InspectionMission(Node):

    def __init__(self):
        super().__init__('inspection_mission')

        # Parameters
        self.declare_parameter('target_depth', 20.0)             # m
        self.declare_parameter('target_speed', 1.0)              # m/s (default single mission speed)
        self.declare_parameter('inspection_speed', 1.0)          # m/s (alias/fallback)
        self.declare_parameter('target_heading', 90.0)           # deg
        self.declare_parameter('recovery_heading', 0.0)          # deg (default surfacing heading)
        self.declare_parameter('total_mission_duration', 1800.0) # s (30 min)
        self.declare_parameter('surface_threshold', 0.3)         # m
        self.declare_parameter('depth_tolerance', 0.5)           # m
        self.declare_parameter('auto_start', False)              # start on launch if True

        self.target_depth = float(self.get_parameter('target_depth').value)
        self.target_speed = float(self.get_parameter('target_speed').value)
        self.target_heading = float(self.get_parameter('target_heading').value)
        self.recovery_heading = float(self.get_parameter('recovery_heading').value)
        self.total_mission_duration = float(self.get_parameter('total_mission_duration').value)
        self.surface_threshold = float(self.get_parameter('surface_threshold').value)
        self.depth_tolerance = float(self.get_parameter('depth_tolerance').value)

        # Vehicle Telemetry State
        self.current_depth = 0.0
        self.current_heading = 0.0
        self.current_speed = 0.0  # m/s
        self.max_depth_reached = 0.0

        # Mission State Machine
        self.state = "IDLE"
        self.start_time = None
        self.elapsed_time = 0.0
        self.stop_requested = False

        self.last_published_depth = None
        self.last_published_heading = None

        # Speed Controller PID (output throttle command [0.0 to 1.0])
        self.speed_pid = PID(0.5, 0.05, 0.01, 0.0, 1.0)
        self.prev_speed_time = self.get_clock().now()

        # Subscriptions
        self.create_subscription(Depth, '/auv/depth', self.depth_callback, 10)
        self.create_subscription(Orientation, '/auv/orientation', self.orientation_callback, 10)
        self.create_subscription(Odometry, '/auv/ideal_state', self.ideal_state_callback, 10)
        self.create_subscription(TwistStamped, '/auv/dvl/velocity', self.dvl_vel_callback, 10)
        self.create_subscription(Mission, '/auv/mission', self.mission_command_callback, 10)

        # Publishers
        self.arm_pub = self.create_publisher(Bool, '/auv/arm_cmd', 10)
        self.desired_depth_pub = self.create_publisher(Float32, '/auv/desired_depth', 10)
        self.desired_heading_pub = self.create_publisher(Float32, '/auv/desired_heading', 10)
        self.throttle_pub = self.create_publisher(Float32, '/auv/throttle_cmd', 10)
        self.dist_hold_enable_pub = self.create_publisher(Float32, '/auv/distance_hold_enabled', 10)
        self.depth_hold_enable_pub = self.create_publisher(Float32, '/auv/depth_hold_enabled', 10)
        self.heading_hold_enable_pub = self.create_publisher(Float32, '/auv/heading_hold_enabled', 10)

        self.mission_status_pub = self.create_publisher(String, '/auv/mission_status', 10)
        self.telemetry_pub = self.create_publisher(String, '/auv/inspection_mission_telemetry', 10)

        # Timer loop at 20 Hz (0.05s)
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info(
            f"Inspection Mission Node Started | Target Depth: {self.target_depth:.1f}m, "
            f"Target Speed: {self.target_speed:.2f}m/s, Target Heading: {self.target_heading:.1f}deg, "
            f"Recovery Heading: {self.recovery_heading:.1f}deg, Duration: {self.total_mission_duration:.1f}s"
        )

        if self.get_parameter('auto_start').value:
            self.start_mission()

    def publish_target_depth(self, depth: float):
        """Publishes desired depth only when target value changes to avoid resetting Depth PID integral."""
        if self.last_published_depth is None or abs(self.last_published_depth - depth) > 1e-3:
            self.last_published_depth = depth
            des_depth = Float32()
            des_depth.data = float(depth)
            self.desired_depth_pub.publish(des_depth)

    def publish_target_heading(self, heading: float):
        """Publishes desired heading only when target value changes."""
        if self.last_published_heading is None or abs(self.last_published_heading - heading) > 1e-3:
            self.last_published_heading = heading
            des_hdg = Float32()
            des_hdg.data = float(heading)
            self.desired_heading_pub.publish(des_hdg)

    def depth_callback(self, msg: Depth):
        self.current_depth = float(msg.depth)
        self.max_depth_reached = max(self.max_depth_reached, self.current_depth)

    def orientation_callback(self, msg: Orientation):
        self.current_heading = float(msg.yaw)

    def ideal_state_callback(self, msg: Odometry):
        self.current_depth = float(msg.z)
        self.current_heading = float(msg.yaw)
        self.current_speed = float(msg.vx)
        self.max_depth_reached = max(self.max_depth_reached, self.current_depth)

    def dvl_vel_callback(self, msg: TwistStamped):
        self.current_speed = float(msg.twist.linear.x)

    def mission_command_callback(self, msg: Mission):
        if msg.cancel or (msg.start and msg.duration < 0):
            self.stop_mission()
            return

        if msg.depth > 0:
            self.target_depth = float(msg.depth)
        if msg.heading != 0.0 or msg.depth > 0:
            self.target_heading = float(msg.heading)
        if msg.inspection_speed > 0:
            self.target_speed = float(msg.inspection_speed)
        if msg.duration > 0:
            duration_val = float(msg.duration)
            self.total_mission_duration = duration_val * 60.0 if duration_val <= 120.0 else duration_val

        # Only transition from IDLE to DESCENDING (active autonomy) when explicitly commanded to start
        if msg.start:
            self.start_mission()

    def start_mission(self):
        """Creates a completely NEW mission with timer starting at t=00:00."""
        self.start_time = self.get_clock().now()
        self.elapsed_time = 0.0
        self.stop_requested = False
        self.state = "DESCENDING"
        self.max_depth_reached = self.current_depth
        self.speed_pid.integral = 0.0
        self.speed_pid.prev_error = 0.0
        self.prev_speed_time = self.get_clock().now()
        self.last_published_depth = None
        self.last_published_heading = None

        # Arm the vehicle so auv_control processes actuator commands
        arm_msg = Bool()
        arm_msg.data = True
        self.arm_pub.publish(arm_msg)

        # Enable Depth & Heading hold
        dh_enable = Float32()
        dh_enable.data = 1.0
        self.depth_hold_enable_pub.publish(dh_enable)

        hh_enable = Float32()
        hh_enable.data = 1.0
        self.heading_hold_enable_pub.publish(hh_enable)

        # Enable Speed/Throttle hold
        dist_enable = Float32()
        dist_enable.data = 1.0
        self.dist_hold_enable_pub.publish(dist_enable)

        # Publish initial targets
        self.publish_target_depth(self.target_depth)
        self.publish_target_heading(self.target_heading)

        self.get_logger().info(
            f"NEW MISSION STARTED (t=00:00) | Target Depth: {self.target_depth:.1f}m, "
            f"Target Speed: {self.target_speed:.2f}m/s, Heading: {self.target_heading:.1f}deg, "
            f"Duration: {self.total_mission_duration/60.0:.1f}min"
        )

    def stop_mission(self):
        """Aborts current mission and commands vehicle to surface actively if underwater."""
        self.start_time = None
        self.elapsed_time = 0.0
        self.stop_requested = True

        if self.current_depth <= self.surface_threshold:
            self.state = "IDLE"
            self.stop_requested = False

            # Neutral commands & disarm controllers
            throttle_msg = Float32()
            throttle_msg.data = 0.0
            self.throttle_pub.publish(throttle_msg)

            dist_enable = Float32()
            dist_enable.data = 0.0
            self.dist_hold_enable_pub.publish(dist_enable)

            dh_enable = Float32()
            dh_enable.data = 0.0
            self.depth_hold_enable_pub.publish(dh_enable)

            hh_enable = Float32()
            hh_enable.data = 0.0
            self.heading_hold_enable_pub.publish(hh_enable)

            status_msg = String()
            status_msg.data = "IDLE"
            self.mission_status_pub.publish(status_msg)

            self.get_logger().info("STOP MISSION AT SURFACE: Cleared mission, state set to IDLE.")
        else:
            self.state = "SURFACING"

            # Immediately publish surfacing depth (0.0m) & recovery heading targets
            self.publish_target_depth(0.0)
            self.publish_target_heading(self.recovery_heading)

            self.get_logger().info(
                f"STOP MISSION UNDERWATER (Depth: {self.current_depth:.2f}m): "
                f"Cleared mission progress, active surfacing initiated to 0.0m with speed {self.target_speed:.2f}m/s and recovery heading {self.recovery_heading:.1f}°"
            )

    def compute_speed_throttle(self, target_speed: float, dt: float) -> float:
        """Computes throttle command [0.0 to 1.0] for target forward speed (m/s)."""
        if target_speed <= 0.0:
            return 0.0

        # Hydrodynamic drag model feedforward: F_drag = 15.0*v + 25.0*v^2, F_thrust = 120.0*T^2
        f_drag = 15.0 * target_speed + 25.0 * (target_speed ** 2)
        ff_throttle = math.sqrt(max(0.0, f_drag / 120.0))

        # Feedback correction
        error = target_speed - self.current_speed
        fb_output = self.speed_pid.update(error, dt)

        commanded_throttle = max(0.0, min(1.0, ff_throttle + fb_output))
        return commanded_throttle

    def control_loop(self):
        now = self.get_clock().now()

        if self.start_time is not None and self.state in ("DESCENDING", "INSPECTION"):
            calc_elapsed = (now - self.start_time).nanoseconds / 1e9
            if calc_elapsed > self.elapsed_time:
                self.elapsed_time = calc_elapsed
        elif self.state == "SURFACING" and self.stop_requested:
            self.elapsed_time = 0.0

        dt = (now - self.prev_speed_time).nanoseconds / 1e9
        self.prev_speed_time = now
        if dt <= 0.0:
            dt = 0.05

        commanded_speed = 0.0
        active_target_depth = self.target_depth
        active_target_heading = self.target_heading
        depth_pid_active = False

        # --- STATE MACHINE ---
        if self.state == "IDLE":
            depth_pid_active = False
            commanded_speed = 0.0
            throttle_val = 0.0
            active_target_depth = 0.0
            active_target_heading = self.current_heading

        elif self.state == "DESCENDING":
            depth_pid_active = True
            active_target_depth = self.target_depth
            active_target_heading = self.target_heading
            commanded_speed = self.target_speed

            self.publish_target_depth(active_target_depth)
            self.publish_target_heading(active_target_heading)

            throttle_val = self.compute_speed_throttle(commanded_speed, dt)

            dist_enable = Float32()
            dist_enable.data = 1.0
            self.dist_hold_enable_pub.publish(dist_enable)

            throttle_msg = Float32()
            throttle_msg.data = throttle_val
            self.throttle_pub.publish(throttle_msg)

            # Check mission duration expiry during descent
            if self.elapsed_time >= self.total_mission_duration:
                self.get_logger().info(
                    f"MISSION DURATION EXPIRED DURING DESCENT (Elapsed: {self.elapsed_time:.1f}s) -> Transitioning to SURFACING"
                )
                self.state = "SURFACING"
                self.publish_target_depth(0.0)
                self.publish_target_heading(self.recovery_heading)

            # Check target depth reached
            elif self.current_depth >= (self.target_depth - self.depth_tolerance):
                self.get_logger().info(
                    f"TARGET DEPTH REACHED ({self.current_depth:.2f} m at t={self.elapsed_time:.1f}s) -> Transitioning to INSPECTION"
                )
                self.state = "INSPECTION"

        elif self.state == "INSPECTION":
            depth_pid_active = True
            active_target_depth = self.target_depth
            active_target_heading = self.target_heading
            commanded_speed = self.target_speed

            self.publish_target_depth(active_target_depth)
            self.publish_target_heading(active_target_heading)

            throttle_val = self.compute_speed_throttle(commanded_speed, dt)

            dist_enable = Float32()
            dist_enable.data = 1.0
            self.dist_hold_enable_pub.publish(dist_enable)

            throttle_msg = Float32()
            throttle_msg.data = throttle_val
            self.throttle_pub.publish(throttle_msg)

            # Check mission duration timer
            if self.elapsed_time >= self.total_mission_duration:
                self.get_logger().info(
                    f"TOTAL MISSION TIME EXPIRED ({self.elapsed_time:.1f}s >= {self.total_mission_duration:.1f}s) -> Transitioning to SURFACING"
                )
                self.state = "SURFACING"
                self.publish_target_depth(0.0)
                self.publish_target_heading(self.recovery_heading)

        elif self.state == "SURFACING":
            depth_pid_active = True
            active_target_depth = 0.0
            active_target_heading = self.recovery_heading
            commanded_speed = self.target_speed

            self.publish_target_depth(active_target_depth)
            self.publish_target_heading(active_target_heading)

            throttle_val = self.compute_speed_throttle(commanded_speed, dt)

            dist_enable = Float32()
            dist_enable.data = 1.0
            self.dist_hold_enable_pub.publish(dist_enable)

            throttle_msg = Float32()
            throttle_msg.data = throttle_val
            self.throttle_pub.publish(throttle_msg)

            # Check surface threshold reached
            if self.current_depth <= self.surface_threshold:
                if self.stop_requested:
                    self.get_logger().info(
                        f"SURFACE REACHED AFTER STOP (Depth: {self.current_depth:.2f} m) -> Transitioning to IDLE"
                    )
                    self.state = "IDLE"
                    self.stop_requested = False
                else:
                    self.get_logger().info(
                        f"SURFACE REACHED (Depth: {self.current_depth:.2f} m) -> Transitioning to MISSION COMPLETE"
                    )
                    self.state = "MISSION COMPLETE"

        elif self.state == "MISSION COMPLETE":
            depth_pid_active = False
            commanded_speed = 0.0
            active_target_depth = 0.0
            active_target_heading = self.recovery_heading

            # Propulsion & Control surfaces neutral
            throttle_val = 0.0
            throttle_msg = Float32()
            throttle_msg.data = 0.0
            self.throttle_pub.publish(throttle_msg)

            dist_enable = Float32()
            dist_enable.data = 0.0
            self.dist_hold_enable_pub.publish(dist_enable)

            # Turn off depth & heading hold
            dh_enable = Float32()
            dh_enable.data = 0.0
            self.depth_hold_enable_pub.publish(dh_enable)

            hh_enable = Float32()
            hh_enable.data = 0.0
            self.heading_hold_enable_pub.publish(hh_enable)

        # Publish Mission Status String
        status_msg = String()
        status_msg.data = self.state
        self.mission_status_pub.publish(status_msg)

        # Format Telemetry JSON for HUD
        m_curr = int(self.elapsed_time) // 60
        s_curr = int(self.elapsed_time) % 60
        m_tot = int(self.total_mission_duration) // 60
        s_tot = int(self.total_mission_duration) % 60
        time_str = f"{m_curr:02d}:{s_curr:02d} / {m_tot:02d}:{s_tot:02d}"

        telem_data = {
            "status": self.state,
            "target_depth": active_target_depth,
            "current_depth": self.current_depth,
            "target_speed": self.target_speed,
            "commanded_speed": commanded_speed,
            "current_speed": self.current_speed,
            "target_heading": active_target_heading,
            "current_heading": self.current_heading,
            "recovery_heading": self.recovery_heading,
            "elapsed_time": self.elapsed_time,
            "total_duration": self.total_mission_duration,
            "time_str": time_str,
            "depth_pid_active": depth_pid_active
        }

        telem_msg = String()
        telem_msg.data = json.dumps(telem_data)
        self.telemetry_pub.publish(telem_msg)


def main(args=None):
    rclpy.init(args=args)
    node = InspectionMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.stop_mission()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()