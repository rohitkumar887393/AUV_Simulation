#!/usr/bin/env python3

import sys
import math
import rclpy
from rclpy.node import Node

# 1 & 2. Set backend to TkAgg BEFORE importing pyplot
try:
    import matplotlib
    matplotlib.use("TkAgg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
except Exception as e:
    print(f"[visualizer_3d] ERROR: Failed to initialize Matplotlib backend: {e}", file=sys.stderr)
    sys.exit(1)

from auv_interfaces.msg import Odometry, Orientation, ActuatorCmds


class Visualizer3D(Node):

    def __init__(self):
        super().__init__('visualizer_3d')

        # State Variables
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0  # Depth (+z = DOWN)
        self.roll = 0.0   # deg
        self.pitch = 0.0  # deg
        self.yaw = 0.0    # deg
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0

        # Actuators
        self.act_el_l = 1500
        self.act_el_r = 1500
        self.act_rud_l = 1500
        self.act_rud_r = 1500
        self.act_thruster = 1500

        # Trajectory Buffer
        self.history_x = []
        self.history_y = []
        self.history_z = []
        self.max_history = 500

        self.first_state_received = False

        # 11. Subscriptions
        self.create_subscription(Odometry, '/auv/ideal_state', self.ideal_state_cb, 10)
        self.create_subscription(Orientation, '/auv/orientation', self.orientation_cb, 10)
        self.create_subscription(ActuatorCmds, '/auv/actual_actuator_cmds', self.actuator_cb, 10)

    def ideal_state_cb(self, msg: Odometry):
        if not self.first_state_received:
            self.first_state_received = True
            print("[visualizer_3d] Received /auv/ideal_state")
            self.get_logger().info("[visualizer_3d] Received /auv/ideal_state")

        self.x = float(msg.x)
        self.y = float(msg.y)
        self.z = float(msg.z)
        self.roll = float(msg.roll)
        self.pitch = float(msg.pitch)
        self.yaw = float(msg.yaw)
        self.vx = float(msg.vx)
        self.vy = float(msg.vy)
        self.vz = float(msg.vz)

        self.history_x.append(self.x)
        self.history_y.append(self.y)
        self.history_z.append(self.z)
        if len(self.history_x) > self.max_history:
            self.history_x.pop(0)
            self.history_y.pop(0)
            self.history_z.pop(0)

    def orientation_cb(self, msg: Orientation):
        self.roll = float(msg.roll)
        self.pitch = float(msg.pitch)
        self.yaw = float(msg.yaw)

    def actuator_cb(self, msg: ActuatorCmds):
        self.act_el_l = msg.elevator_left
        self.act_el_r = msg.elevator_right
        self.act_rud_l = msg.rudder_left
        self.act_rud_r = msg.rudder_right
        self.act_thruster = msg.main_thruster

    def get_rotation_matrix(self):
        r = math.radians(self.roll)
        p = math.radians(self.pitch)
        y = math.radians(self.yaw)

        cr, sr = math.cos(r), math.sin(r)
        cp, sp = math.cos(p), math.sin(p)
        cy, sy = math.cos(y), math.sin(y)

        # R = Rz(yaw) * Ry(pitch) * Rx(roll)
        R = [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp,     cp * sr,               cp * cr]
        ]
        return R

    def transform_point(self, px, py, pz, R):
        wx = self.x + R[0][0] * px + R[0][1] * py + R[0][2] * pz
        wy = self.y + R[1][0] * px + R[1][1] * py + R[1][2] * pz
        wz = self.z + R[2][0] * px + R[2][1] * py + R[2][2] * pz
        return wx, wy, wz

    def update_plot(self, ax):
        ax.clear()

        # 12. Underwater environment setup
        ax.set_facecolor('#07131e')
        ax.set_title("3D AUV Autonomous Simulation Viewer", color='white', fontsize=12, fontweight='bold')

        ax.set_xlabel('X (North) [m]', color='white')
        ax.set_ylabel('Y (East) [m]', color='white')
        ax.set_zlabel('Z (Depth) [m]', color='white')
        ax.tick_params(colors='white')
        ax.xaxis.pane.set_edgecolor('#1c3144')
        ax.yaxis.pane.set_edgecolor('#1c3144')
        ax.zaxis.pane.set_edgecolor('#1c3144')
        ax.xaxis.pane.fill = False
        ax.yaxis.pane.fill = False
        ax.zaxis.pane.fill = False
        ax.grid(True, color='#1c3144', linestyle='--', linewidth=0.5)

        # Invert Z axis (+z depth downwards)
        ax.invert_zaxis()

        # Water Surface (Z=0) & Seabed (Z=15) Planes
        grid_range_x = [-5, max(15, self.x + 5)]
        grid_range_y = [-5, max(15, self.y + 5)]

        # Water Surface line grid
        ax.plot([grid_range_x[0], grid_range_x[1], grid_range_x[1], grid_range_x[0], grid_range_x[0]],
                [grid_range_y[0], grid_range_y[0], grid_range_y[1], grid_range_y[1], grid_range_y[0]],
                [0, 0, 0, 0, 0], color='#00a8e8', linestyle=':', alpha=0.5, label='Water Surface (Z=0m)')

        # Seabed line grid
        seabed_z = max(15.0, self.z + 5.0)
        ax.plot([grid_range_x[0], grid_range_x[1], grid_range_x[1], grid_range_x[0], grid_range_x[0]],
                [grid_range_y[0], grid_range_y[0], grid_range_y[1], grid_range_y[1], grid_range_y[0]],
                [seabed_z, seabed_z, seabed_z, seabed_z, seabed_z], color='#8b5a2b', linestyle=':', alpha=0.4, label=f'Seabed (Z={seabed_z:.0f}m)')

        # Trajectory line
        if len(self.history_x) > 1:
            ax.plot(self.history_x, self.history_y, self.history_z, color='#00ffcc', linewidth=1.5, label='Trajectory')

        # Start Marker
        ax.scatter([0], [0], [0], color='#00ff00', s=40, label='Origin (0,0,0)')

        # 9 & 13. Render 3D AUV Model with Rotation Matrix
        R = self.get_rotation_matrix()

        # Cylindrical Body
        length = 1.2
        radius = 0.16
        num_seg = 10
        body_circles = []
        for x_local in [-0.5, 0.5]:
            circle_pts = []
            for i in range(num_seg):
                ang = 2 * math.pi * i / num_seg
                by = radius * math.cos(ang)
                bz = radius * math.sin(ang)
                circle_pts.append(self.transform_point(x_local, by, bz, R))
            body_circles.append(circle_pts)

        # Draw body longitudinal lines
        for i in range(num_seg):
            p1 = body_circles[0][i]
            p2 = body_circles[1][i]
            ax.plot([p1[0], p2[0]], [p1[1], p2[1]], [p1[2], p2[2]], color='#d90429', linewidth=1.2)

        # Tapered Nose
        nose_tip = self.transform_point(0.85, 0, 0, R)
        for i in range(num_seg):
            p = body_circles[1][i]
            ax.plot([p[0], nose_tip[0]], [p[1], nose_tip[1]], [p[2], nose_tip[2]], color='#ef233c', linewidth=1.5)

        # Tapered Rear Section
        tail_center = self.transform_point(-0.75, 0, 0, R)
        for i in range(num_seg):
            p = body_circles[0][i]
            ax.plot([p[0], tail_center[0]], [p[1], tail_center[1]], [p[2], tail_center[2]], color='#8d99ae', linewidth=1.2)

        # Four Fins (Elevators & Rudders)
        fin_deflect_el = (float(self.act_el_l - 1500) / 400.0) * 0.15
        fin_deflect_rud = (float(self.act_rud_l - 1500) / 400.0) * 0.15

        # Elevator Left / Right
        el_left = [self.transform_point(-0.5, 0.15, 0, R), self.transform_point(-0.75, 0.4, -fin_deflect_el, R)]
        el_right = [self.transform_point(-0.5, -0.15, 0, R), self.transform_point(-0.75, -0.4, fin_deflect_el, R)]
        ax.plot([el_left[0][0], el_left[1][0]], [el_left[0][1], el_left[1][1]], [el_left[0][2], el_left[1][2]], color='#ffb703', linewidth=2.5)
        ax.plot([el_right[0][0], el_right[1][0]], [el_right[0][1], el_right[1][1]], [el_right[0][2], el_right[1][2]], color='#ffb703', linewidth=2.5)

        # Rudder Top / Bottom
        rud_top = [self.transform_point(-0.5, 0, -0.15, R), self.transform_point(-0.75, fin_deflect_rud, -0.4, R)]
        rud_bottom = [self.transform_point(-0.5, 0, 0.15, R), self.transform_point(-0.75, -fin_deflect_rud, 0.4, R)]
        ax.plot([rud_top[0][0], rud_top[1][0]], [rud_top[0][1], rud_top[1][1]], [rud_top[0][2], rud_top[1][2]], color='#fb8500', linewidth=2.5)
        ax.plot([rud_bottom[0][0], rud_bottom[1][0]], [rud_bottom[0][1], rud_bottom[1][0]], [rud_bottom[0][2], rud_bottom[1][2]], color='#fb8500', linewidth=2.5)

        # Rear Thruster Indicator
        thruster_end = self.transform_point(-0.85, 0, 0, R)
        ax.plot([tail_center[0], thruster_end[0]], [tail_center[1], thruster_end[1]], [tail_center[2], thruster_end[2]], color='#e0aaff', linewidth=3.0, label='Thruster')

        # Heading Indicator Arrow Vector
        heading_vector_end = self.transform_point(1.35, 0, 0, R)
        ax.plot([nose_tip[0], heading_vector_end[0]], [nose_tip[1], heading_vector_end[1]], [nose_tip[2], heading_vector_end[2]], color='#00f5d4', linewidth=2.5, label='Heading')

        # Set 3D Plot Bounds (Centered on AUV)
        box_range = 4.0
        ax.set_xlim(self.x - box_range, self.x + box_range)
        ax.set_ylim(self.y - box_range, self.y + box_range)
        ax.set_zlim(max(-1.0, self.z - box_range), self.z + box_range)

        ax.legend(loc='upper right', facecolor='#0d1b2a', edgecolor='white', labelcolor='white', fontsize=7.5)

        # Telemetry Text Overlay
        telemetry_str = (
            f"AUV Pose: X={self.x:5.2f}m | Y={self.y:5.2f}m | Depth={self.z:5.2f}m\n"
            f"Attitude: Roll={self.roll:5.1f}° | Pitch={self.pitch:5.1f}° | Yaw={self.yaw:5.1f}°\n"
            f"Velocities: Vx={self.vx:4.2f}m/s | Vy={self.vy:4.2f}m/s | Vz={self.vz:4.2f}m/s\n"
            f"Actuators (PWM): Elev L/R={self.act_el_l}/{self.act_el_r} | Rud L/R={self.act_rud_l}/{self.act_rud_r} | Thr={self.act_thruster}"
        )
        ax.text2D(0.02, 0.95, telemetry_str, transform=ax.transAxes, color='white', fontsize=8,
                  fontfamily='monospace', bbox=dict(boxstyle='round', facecolor='#0d1b2a', alpha=0.8, edgecolor='#00a8e8'))


def main(args=None):
    rclpy.init(args=args)
    node = Visualizer3D()

    # 15. Startup Log Requirements
    startup_msg_1 = "[visualizer_3d] 3D AUV Viewer Started"
    startup_msg_2 = f"[visualizer_3d] Matplotlib backend: {matplotlib.get_backend()}"
    startup_msg_3 = "[visualizer_3d] Waiting for /auv/ideal_state"

    print(startup_msg_1)
    print(startup_msg_2)
    print(startup_msg_3)

    node.get_logger().info(startup_msg_1)
    node.get_logger().info(startup_msg_2)
    node.get_logger().info(startup_msg_3)

    # 6 & 7. Initialize single interactive Matplotlib figure
    plt.ion()
    fig = plt.figure(figsize=(9, 6.5))
    try:
        fig.canvas.manager.set_window_title('3D AUV Autonomous Simulation Viewer')
    except Exception:
        pass

    ax = fig.add_subplot(111, projection='3d')
    fig.show()

    # 4 & 6. Non-blocking GUI event loop
    try:
        while rclpy.ok() and plt.fignum_exists(fig.number):
            rclpy.spin_once(node, timeout_sec=0.01)
            node.update_plot(ax)
            fig.canvas.draw_idle()
            fig.canvas.flush_events()
            plt.pause(0.01)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"[visualizer_3d] ERROR during execution: {e}", file=sys.stderr)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
