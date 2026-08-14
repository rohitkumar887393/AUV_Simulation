#!/usr/bin/env python3

import os
import sys
import time
import math
import json
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, String
from auv_interfaces.msg import ActuatorCmds, Odometry

# Attempt matplotlib import
try:
    import matplotlib
    if os.environ.get('DISPLAY') is None and sys.platform.startswith('linux'):
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    HAS_MATPLOTLIB = True
except Exception:
    HAS_MATPLOTLIB = False


class SimulationVisualizer(Node):

    def __init__(self):
        super().__init__('simulation_visualizer')

        self.declare_parameter('enable_gui', True)
        self.enable_gui = self.get_parameter('enable_gui').value and HAS_MATPLOTLIB

        # Telemetry Data Storage
        self.history_time = []
        self.history_x = []
        self.history_y = []
        self.history_depth = []
        self.history_target_depth = []
        self.history_yaw = []
        self.history_target_yaw = []

        self.start_time = time.time()

        # Current State
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.r = 0.0

        self.target_heading = 0.0
        self.target_depth = 0.0
        self.target_distance = 0.0
        self.heading_error = 0.0
        self.heading_pid_output = 0.0

        self.heading_hold_status = "OFF"
        self.depth_hold_status = "OFF"
        self.distance_hold_status = "OFF"
        self.mission_status = "IDLE"

        # Commanded PWMs
        self.cmd_el_l = 1500
        self.cmd_el_r = 1500
        self.cmd_rud_l = 1500
        self.cmd_rud_r = 1500
        self.cmd_thruster = 1500

        # Actual Actuator PWMs
        self.act_el_l = 1500
        self.act_el_r = 1500
        self.act_rud_l = 1500
        self.act_rud_r = 1500
        self.act_thruster = 1500

        # Subscriptions
        self.create_subscription(Odometry, '/auv/ideal_state', self.state_cb, 10)
        self.create_subscription(ActuatorCmds, '/auv/actuator_cmds', self.cmd_act_cb, 10)
        self.create_subscription(ActuatorCmds, '/auv/actual_actuator_cmds', self.actual_act_cb, 10)

        self.create_subscription(Float32, '/auv/desired_heading', self.des_heading_cb, 10)
        self.create_subscription(Float32, '/auv/desired_depth', self.des_depth_cb, 10)
        self.create_subscription(Float32, '/auv/desired_distance', self.des_dist_cb, 10)
        self.create_subscription(String, '/auv/heading_hold_status', self.hh_stat_cb, 10)
        self.create_subscription(String, '/auv/depth_hold_status', self.dh_stat_cb, 10)
        self.create_subscription(String, '/auv/distance_hold_status', self.dist_stat_cb, 10)
        self.create_subscription(String, '/auv/mission_status', self.miss_stat_cb, 10)
        self.create_subscription(String, '/auv/heading_telemetry', self.heading_telem_cb, 10)

        # Setup GUI Plot if enabled
        if self.enable_gui:
            try:
                plt.ion()
                self.fig, self.axs = plt.subplots(2, 2, figsize=(10, 8))
                self.fig.suptitle('AUV Simulation Live Visualization & Telemetry Dashboard', fontsize=12, fontweight='bold')
                plt.show(block=False)
                self.gui_timer = self.create_timer(0.1, self.update_gui)
            except Exception as e:
                self.get_logger().warn(f'Could not initialize matplotlib GUI window: {e}. Falling back to console telemetry updates.')
                self.enable_gui = False

        if not self.enable_gui:
            self.console_timer = self.create_timer(1.0, self.update_console)

        self.get_logger().info(f'Simulation Visualizer Node Started (GUI Enabled={self.enable_gui})')

    def state_cb(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.z = msg.z
        self.roll = msg.roll
        self.pitch = msg.pitch
        self.yaw = msg.yaw
        self.vx = msg.vx
        self.vy = msg.vy
        self.vz = msg.vz

        t_now = time.time() - self.start_time
        self.history_time.append(t_now)
        self.history_x.append(self.x)
        self.history_y.append(self.y)
        self.history_depth.append(self.z)
        self.history_target_depth.append(self.target_depth)
        self.history_yaw.append(self.yaw)
        self.history_target_yaw.append(self.target_heading)

        if len(self.history_time) > 1000:
            self.history_time.pop(0)
            self.history_x.pop(0)
            self.history_y.pop(0)
            self.history_depth.pop(0)
            self.history_target_depth.pop(0)
            self.history_yaw.pop(0)
            self.history_target_yaw.pop(0)

    def cmd_act_cb(self, msg):
        self.cmd_el_l = msg.elevator_left
        self.cmd_el_r = msg.elevator_right
        self.cmd_rud_l = msg.rudder_left
        self.cmd_rud_r = msg.rudder_right
        self.cmd_thruster = msg.main_thruster

    def actual_act_cb(self, msg):
        self.act_el_l = msg.elevator_left
        self.act_el_r = msg.elevator_right
        self.act_rud_l = msg.rudder_left
        self.act_rud_r = msg.rudder_right
        self.act_thruster = msg.main_thruster

    def des_heading_cb(self, msg):
        self.target_heading = msg.data

    def des_depth_cb(self, msg):
        self.target_depth = msg.data

    def des_dist_cb(self, msg):
        self.target_distance = msg.data

    def hh_stat_cb(self, msg):
        self.heading_hold_status = msg.data

    def dh_stat_cb(self, msg):
        self.depth_hold_status = msg.data

    def dist_stat_cb(self, msg):
        self.distance_hold_status = msg.data

    def miss_stat_cb(self, msg):
        self.mission_status = msg.data

    def heading_telem_cb(self, msg):
        try:
            data = json.loads(msg.data)
            self.target_heading = data.get("target", self.target_heading)
            self.heading_error = data.get("error", self.heading_error)
            self.heading_pid_output = data.get("output", self.heading_pid_output)
        except Exception:
            pass

    def update_console(self):
        dashboard = (
            f"\n======================================================================\n"
            f"                     AUV LIVE TELEMETRY DASHBOARD                     \n"
            f"======================================================================\n"
            f" Position   : X: {self.x:6.2f} m | Y: {self.y:6.2f} m | Depth: {self.z:6.2f} m\n"
            f" Orient     : Roll: {self.roll:5.1f}° | Pitch: {self.pitch:5.1f}° | Yaw: {self.yaw:5.1f}°\n"
            f" Velocity   : Forward Vx: {self.vx:5.2f} m/s | Heave Vz: {self.vz:5.2f} m/s\n"
            f" Heading Signal Chain: Target: {self.target_heading:5.1f}° | Current: {self.yaw:5.1f}° | Error: {self.heading_error:5.1f}° | PID Out: {self.heading_pid_output:6.1f} | Rudder PWM: {self.cmd_rud_l:4d} us\n"
            f" Targets    : Desired Heading: {self.target_heading:5.1f}° | Desired Depth: {self.target_depth:5.2f} m | Desired Dist: {self.target_distance:5.2f} m\n"
            f" Controllers: Heading Hold: {self.heading_hold_status:<10} | Depth Hold: {self.depth_hold_status:<10} | Distance Hold: {self.distance_hold_status:<10} | Mission: {self.mission_status:<10}\n"
            f" Actuators (PWM us) [Cmd vs Actual]:\n"
            f"   Elevator L : Cmd: {self.cmd_el_l:4d} us | Actual: {self.act_el_l:4d} us\n"
            f"   Elevator R : Cmd: {self.cmd_el_r:4d} us | Actual: {self.act_el_r:4d} us\n"
            f"   Rudder L   : Cmd: {self.cmd_rud_l:4d} us | Actual: {self.act_rud_l:4d} us\n"
            f"   Rudder R   : Cmd: {self.cmd_rud_r:4d} us | Actual: {self.act_rud_r:4d} us\n"
            f"   Thruster   : Cmd: {self.cmd_thruster:4d} us | Actual: {self.act_thruster:4d} us\n"
            f"======================================================================\n"
        )
        self.get_logger().info(dashboard)

    def update_gui(self):
        if not self.enable_gui or len(self.history_time) == 0:
            return

        try:
            for ax in self.axs.flat:
                ax.clear()

            # Subplot 1: Top-Down Trajectory View (X-Y)
            ax_top = self.axs[0, 0]
            ax_top.plot(self.history_x, self.history_y, 'b-', label='AUV Path')
            ax_top.plot(0, 0, 'go', label='Start (0,0)')
            ax_top.plot(self.x, self.y, 'ro', label='Current Pos')

            yaw_rad = math.radians(self.yaw)
            arrow_len = 0.5
            ax_top.arrow(self.x, self.y, arrow_len * math.cos(yaw_rad), arrow_len * math.sin(yaw_rad),
                         head_width=0.2, head_length=0.2, fc='red', ec='red')

            ax_top.set_title('Top-Down 2D Trajectory View (North/East)')
            ax_top.set_xlabel('X Position (m)')
            ax_top.set_ylabel('Y Position (m)')
            ax_top.grid(True)
            ax_top.legend(loc='upper left', fontsize=8)

            # Subplot 2: Depth vs Time View
            ax_depth = self.axs[0, 1]
            ax_depth.plot(self.history_time, self.history_depth, 'b-', label='Actual Depth')
            ax_depth.plot(self.history_time, self.history_target_depth, 'r--', label='Target Depth')
            ax_depth.set_title('Depth Profile (m)')
            ax_depth.set_xlabel('Time (s)')
            ax_depth.set_ylabel('Depth (m)')
            ax_depth.invert_yaxis()
            ax_depth.grid(True)
            ax_depth.legend(loc='upper right', fontsize=8)

            # Subplot 3: Heading vs Time View
            ax_head = self.axs[1, 0]
            ax_head.plot(self.history_time, self.history_yaw, 'b-', label='Actual Yaw')
            ax_head.plot(self.history_time, self.history_target_yaw, 'r--', label='Target Yaw')
            ax_head.set_title('Heading Profile (deg)')
            ax_head.set_xlabel('Time (s)')
            ax_head.set_ylabel('Heading (deg)')
            ax_head.grid(True)
            ax_head.legend(loc='upper right', fontsize=8)

            # Subplot 4: Telemetry & Actuator Response Panel
            ax_txt = self.axs[1, 1]
            ax_txt.axis('off')
            telemetry_str = (
                f"LIVE TELEMETRY DASHBOARD\n\n"
                f"Position: X={self.x:.2f}m, Y={self.y:.2f}m, Z={self.z:.2f}m\n"
                f"Orient: Roll={self.roll:.1f}°, Pitch={self.pitch:.1f}°, Yaw={self.yaw:.1f}°\n"
                f"Velocities: Vx={self.vx:.2f} m/s, Vz={self.vz:.2f} m/s\n\n"
                f"HEADING SIGNAL CHAIN:\n"
                f"  Target: {self.target_heading:.1f}° | Current: {self.yaw:.1f}°\n"
                f"  Error:  {self.heading_error:.1f}° | PID Out: {self.heading_pid_output:.1f}\n"
                f"  Rudder PWM: {self.cmd_rud_l} us\n\n"
                f"TARGETS:\n"
                f"  Heading: {self.target_heading:.1f}° | Depth: {self.target_depth:.2f}m | Dist: {self.target_distance:.2f}m\n\n"
                f"ACTUATORS (Cmd vs Actual PWM):\n"
                f"  Elev L: {self.cmd_el_l} vs {self.act_el_l} us\n"
                f"  Elev R: {self.cmd_el_r} vs {self.act_el_r} us\n"
                f"  Rud L:  {self.cmd_rud_l} vs {self.act_rud_l} us\n"
                f"  Rud R:  {self.cmd_rud_r} vs {self.act_rud_r} us\n"
                f"  Thrust: {self.cmd_thruster} vs {self.act_thruster} us\n"
            )
            ax_txt.text(0.05, 0.95, telemetry_str, transform=ax_txt.transAxes,
                        fontsize=8.5, verticalalignment='top', fontfamily='monospace',
                        bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
        except Exception as e:
            self.get_logger().debug(f'GUI update exception: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = SimulationVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
