#!/usr/bin/env python3

import math
import json
import rclpy

from rclpy.node import Node

from std_msgs.msg import Float32
from std_msgs.msg import String
from std_srvs.srv import Trigger
from auv_interfaces.msg import Odometry, ActuatorCmds


class IdealAUV(Node):

    def __init__(self):

        super().__init__('ideal_auv')

        # Parameters
        self.declare_parameter('simulation_rate', 50.0)

        self.declare_parameter('servo_time_constant', 0.10)
        self.declare_parameter('thruster_time_constant', 0.20)

        self.declare_parameter('mass', 25.0)
        self.declare_parameter('ix', 0.5)
        self.declare_parameter('iy', 2.0)
        self.declare_parameter('iz', 2.0)

        self.declare_parameter('water_density', 1025.0)
        self.declare_parameter('displaced_volume', 0.0247)
        self.declare_parameter('gravity', 9.81)
        self.declare_parameter('neutral_buoyancy_ratio', 1.000)
        self.declare_parameter('z_bg', 0.03)

        self.declare_parameter('thrust_coefficient', 120.0)

        self.declare_parameter('linear_drag_x', 15.0)
        self.declare_parameter('quadratic_drag_x', 25.0)

        self.declare_parameter('linear_drag_y', 35.0)
        self.declare_parameter('quadratic_drag_y', 60.0)

        self.declare_parameter('linear_drag_z', 15.0)
        self.declare_parameter('quadratic_drag_z', 40.0)

        self.declare_parameter('linear_drag_roll', 2.0)
        self.declare_parameter('quadratic_drag_roll', 5.0)

        self.declare_parameter('linear_drag_pitch', 25.0)
        self.declare_parameter('quadratic_drag_pitch', 40.0)

        self.declare_parameter('linear_drag_yaw', 12.0)
        self.declare_parameter('quadratic_drag_yaw', 35.0)

        self.declare_parameter('elevator_lift_coeff', 0.015)
        self.declare_parameter('rudder_lift_coeff', 0.0035)
        self.declare_parameter('roll_lift_coeff', 0.004)

        self.declare_parameter('maximum_forward_velocity', 4.0)
        self.declare_parameter('maximum_angular_velocity', 60.0)

        # Load parameters
        self.sim_rate = float(self.get_parameter('simulation_rate').value)
        self.tau_servo = float(self.get_parameter('servo_time_constant').value)
        self.tau_thruster = float(self.get_parameter('thruster_time_constant').value)

        self.mass = float(self.get_parameter('mass').value)
        self.ix = float(self.get_parameter('ix').value)
        self.iy = float(self.get_parameter('iy').value)
        self.iz = float(self.get_parameter('iz').value)

        self.rho = float(self.get_parameter('water_density').value)
        self.volume = float(self.get_parameter('displaced_volume').value)
        self.g = float(self.get_parameter('gravity').value)
        self.buoy_ratio = float(self.get_parameter('neutral_buoyancy_ratio').value)
        self.z_bg = float(self.get_parameter('z_bg').value)

        self.Kt = float(self.get_parameter('thrust_coefficient').value)

        self.Xu = float(self.get_parameter('linear_drag_x').value)
        self.Xuu = float(self.get_parameter('quadratic_drag_x').value)

        self.Yv = float(self.get_parameter('linear_drag_y').value)
        self.Yvv = float(self.get_parameter('quadratic_drag_y').value)

        self.Zw = float(self.get_parameter('linear_drag_z').value)
        self.Zww = float(self.get_parameter('quadratic_drag_z').value)

        self.Kp = float(self.get_parameter('linear_drag_roll').value)
        self.Kpp = float(self.get_parameter('quadratic_drag_roll').value)

        self.Kq = float(self.get_parameter('linear_drag_pitch').value)
        self.Kqq = float(self.get_parameter('quadratic_drag_pitch').value)

        self.Kr = float(self.get_parameter('linear_drag_yaw').value)
        self.Krr = float(self.get_parameter('quadratic_drag_yaw').value)

        self.elev_coeff = float(self.get_parameter('elevator_lift_coeff').value)
        self.rud_coeff = float(self.get_parameter('rudder_lift_coeff').value)
        self.roll_coeff = float(self.get_parameter('roll_lift_coeff').value)

        self.max_fwd_vel = float(self.get_parameter('maximum_forward_velocity').value)
        self.max_ang_vel = float(self.get_parameter('maximum_angular_velocity').value)

        self.step_counter = 0

        # Vehicle State: (x, y, z, roll, pitch, yaw, u, v, w, p, q, r)
        self.reset_state()

        # Commanded Actuator PWM Inputs (from /auv/actuator_cmds)
        self.cmd_elevator_left = 1500
        self.cmd_elevator_right = 1500
        self.cmd_rudder_left = 1500
        self.cmd_rudder_right = 1500
        self.cmd_main_thruster = 1500

        # Actual Actuator States (Normalized [-1.0, +1.0], start neutral 0.0)
        self.act_el_l = 0.0
        self.act_el_r = 0.0
        self.act_rud_l = 0.0
        self.act_rud_r = 0.0
        self.act_thruster = 0.0

        # Subscribers & Publishers
        self.create_subscription(
            ActuatorCmds,
            '/auv/actuator_cmds',
            self.actuator_callback,
            1
        )

        self.state_pub = self.create_publisher(
            Odometry,
            '/auv/ideal_state',
            1
        )

        self.actual_act_pub = self.create_publisher(
            ActuatorCmds,
            '/auv/actual_actuator_cmds',
            1
        )

        self.physics_pub = self.create_publisher(
            String,
            '/auv/physics_telemetry',
            1
        )

        # Reset Service (/auv/sim/reset)
        self.reset_service = self.create_service(
            Trigger,
            '/auv/sim/reset',
            self.reset_callback
        )

        dt = 1.0 / self.sim_rate
        self.timer = self.create_timer(dt, self.physics_step)

        effective_volume = self.volume * self.buoy_ratio
        F_weight = self.mass * self.g
        F_buoyancy = - (self.rho * effective_volume * self.g)
        net_buoyancy = F_buoyancy + F_weight

        self.get_logger().info("==================================================")
        self.get_logger().info("RIGOROUS 6-DOF PHYSICS CORE INITIALIZATION")
        self.get_logger().info("==================================================")
        self.get_logger().info(f"MASS:\n{self.mass:.2f} kg\n")
        self.get_logger().info(f"DISPLACED VOLUME:\n{effective_volume:.4f} m^3\n")
        self.get_logger().info(f"WATER DENSITY:\n{self.rho:.1f} kg/m^3\n")
        self.get_logger().info(f"WEIGHT:\n+{F_weight:.1f} N\n")
        self.get_logger().info(f"BUOYANCY:\n{F_buoyancy:.1f} N\n")
        self.get_logger().info(f"NET BUOYANCY:\n{net_buoyancy:.1f} N\n")
        self.get_logger().info("==================================================")

    def reset_state(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.5  # Operational start depth underwater (0.5m)
        self.vz = 0.0 # World vertical velocity (m/s)

        self.roll = 0.0    # deg
        self.pitch = 0.0   # deg
        self.yaw = 0.0     # deg

        self.u = 0.0  # m/s surge (body frame)
        self.v = 0.0  # m/s sway (body frame)
        self.w = 0.0  # m/s heave (body frame)

        self.p = 0.0  # rad/s roll rate (body frame)
        self.q = 0.0  # rad/s pitch rate (body frame)
        self.r = 0.0  # rad/s yaw rate (body frame)

    def reset_callback(self, request, response):
        self.reset_state()
        self.cmd_elevator_left = 1500
        self.cmd_elevator_right = 1500
        self.cmd_rudder_left = 1500
        self.cmd_rudder_right = 1500
        self.cmd_main_thruster = 1500

        self.act_el_l = 0.0
        self.act_el_r = 0.0
        self.act_rud_l = 0.0
        self.act_rud_r = 0.0
        self.act_thruster = 0.0

        response.success = True
        response.message = "6-DOF AUV simulation state and actuators reset to operational underwater state (z=0.5m)"
        self.get_logger().info('SIMULATION RESET TRIGGERED: Vehicle state reset.')
        return response

    def actuator_callback(self, msg):
        self.cmd_elevator_left = msg.elevator_left
        self.cmd_elevator_right = msg.elevator_right
        self.cmd_rudder_left = msg.rudder_left
        self.cmd_rudder_right = msg.rudder_right
        self.cmd_main_thruster = msg.main_thruster

    def physics_step(self):
        dt = 1.0 / self.sim_rate
        self.step_counter += 1

        # 1. Normalize Commanded PWM Inputs [-1.0, +1.0]
        norm_cmd_el_l = (float(self.cmd_elevator_left) - 1500.0) / 400.0
        norm_cmd_el_r = (float(self.cmd_elevator_right) - 1500.0) / 400.0
        norm_cmd_rud_l = (float(self.cmd_rudder_left) - 1500.0) / 400.0
        norm_cmd_rud_r = (float(self.cmd_rudder_right) - 1500.0) / 400.0
        norm_cmd_thruster = (float(self.cmd_main_thruster) - 1500.0) / 400.0

        # 2. First-Order Actuator Spooling Dynamics
        self.act_el_l += ((norm_cmd_el_l - self.act_el_l) / self.tau_servo) * dt
        self.act_el_r += ((norm_cmd_el_r - self.act_el_r) / self.tau_servo) * dt
        self.act_rud_l += ((norm_cmd_rud_l - self.act_rud_l) / self.tau_servo) * dt
        self.act_rud_r += ((norm_cmd_rud_r - self.act_rud_r) / self.tau_servo) * dt
        self.act_thruster += ((norm_cmd_thruster - self.act_thruster) / self.tau_thruster) * dt

        # Clamp actual states strictly to [-1.0, +1.0]
        self.act_el_l = max(-1.0, min(1.0, self.act_el_l))
        self.act_el_r = max(-1.0, min(1.0, self.act_el_r))
        self.act_rud_l = max(-1.0, min(1.0, self.act_rud_l))
        self.act_rud_r = max(-1.0, min(1.0, self.act_rud_r))
        self.act_thruster = max(-1.0, min(1.0, self.act_thruster))

        # Publish Actual Actuator PWMs
        actual_msg = ActuatorCmds()
        actual_msg.elevator_left = int(round(1500.0 + self.act_el_l * 400.0))
        actual_msg.elevator_right = int(round(1500.0 + self.act_el_r * 400.0))
        actual_msg.rudder_left = int(round(1500.0 + self.act_rud_l * 400.0))
        actual_msg.rudder_right = int(round(1500.0 + self.act_rud_r * 400.0))
        actual_msg.main_thruster = int(round(1500.0 + self.act_thruster * 400.0))
        self.actual_act_pub.publish(actual_msg)

        # 3. Control Surface Inputs
        pitch_input = (self.act_el_l - self.act_el_r) / 2.0
        roll_input = (self.act_el_l + self.act_el_r) / 2.0
        rudder_input = (self.act_rud_l - self.act_rud_r) / 2.0

        # 4. Authority Scaling (Speed Authority * Envelope Authority)
        speed_authority = max(0.0, min(1.0, abs(self.u) / 0.5))
        abs_pitch = abs(self.pitch)
        if abs_pitch <= 30.0:
            envelope_authority = 1.0
        elif abs_pitch <= 45.0:
            envelope_authority = 1.0 - 0.9 * ((abs_pitch - 30.0) / 15.0)
        else:
            envelope_authority = 0.1
        
        dynamic_authority = speed_authority * envelope_authority
        q_dyn = 0.5 * self.rho * (self.u ** 2)

        # 5. Hydrostatic Weight & Buoyancy Forces (20 cm Submerged Waterline Equilibrium Trim)
        trim_factor = 0.985 + 0.035 * max(0.0, min(1.0, self.z / 0.20))
        effective_volume = self.volume * self.buoy_ratio * trim_factor
        F_weight = self.mass * self.g                          # + m*g (DOWNWARD = +Z)
        F_buoyancy = - (self.rho * effective_volume * self.g)   # - rho*V*g (UPWARD = -Z)
        F_net_buoyancy = F_buoyancy + F_weight                  # NEGATIVE for positive buoyancy

        roll_rad = math.radians(self.roll)
        pitch_rad = math.radians(self.pitch)
        yaw_rad = math.radians(self.yaw)

        # Hydrostatic Restoring Righting Moments (CG/CB metacentric offset z_bg)
        M_restore = - F_weight * self.z_bg * math.sin(pitch_rad)
        L_restore = - F_weight * self.z_bg * math.sin(roll_rad) * math.cos(pitch_rad)

        # 6. Thruster Force (Surge Body X)
        F_thrust = self.Kt * self.act_thruster * abs(self.act_thruster)

        # 7. Body Frame Drag Forces (Surge, Sway, Heave)
        F_drag_x = - (self.Xu * self.u + self.Xuu * abs(self.u) * self.u)
        F_drag_y = - (self.Yv * self.v + self.Yvv * abs(self.v) * self.v)
        F_drag_z = - (self.Zw * self.w + self.Zww * abs(self.w) * self.w)

        # Hydrostatic Buoyancy/Gravity Projections into Body Frame (Fossen formulation)
        F_buoy_body_x = F_net_buoyancy * math.sin(pitch_rad)
        F_buoy_body_y = F_net_buoyancy * math.sin(roll_rad) * math.cos(pitch_rad)
        F_buoy_body_z = F_net_buoyancy * math.cos(roll_rad) * math.cos(pitch_rad)

        # 8. Body Translational Accelerations & Velocity Integration (Surge, Sway, Heave)
        du_dt = (F_thrust + F_drag_x + F_buoy_body_x) / self.mass
        dv_dt = (F_drag_y + F_buoy_body_y) / self.mass
        dw_dt = (F_drag_z + F_buoy_body_z) / self.mass

        self.u += du_dt * dt
        self.v += dv_dt * dt
        self.w += dw_dt * dt
        self.u = max(-self.max_fwd_vel, min(self.max_fwd_vel, self.u))

        # 9. Control Surface Hydrodynamic Moments with Dynamic Authority Scaling
        M_elevator = self.elev_coeff * q_dyn * pitch_input * dynamic_authority
        N_rudder = self.rud_coeff * q_dyn * rudder_input * dynamic_authority
        L_roll = self.roll_coeff * q_dyn * roll_input * dynamic_authority

        # Angular Hydrodynamic Damping Moments
        M_drag_p = - (self.Kp * self.p + self.Kpp * abs(self.p) * self.p)
        M_drag_q = - (self.Kq * self.q + self.Kqq * abs(self.q) * self.q)
        M_drag_r = - (self.Kr * self.r + self.Krr * abs(self.r) * self.r)

        # Total Pitch, Roll, Yaw Moments & Rotational Accelerations
        M_total = M_elevator + M_restore + M_drag_q
        L_total = L_roll + L_restore + M_drag_p
        N_total = N_rudder + M_drag_r

        dp_dt = L_total / self.ix
        dq_dt = M_total / self.iy
        dr_dt = N_total / self.iz

        self.p += dp_dt * dt
        self.q += dq_dt * dt
        self.r += dr_dt * dt

        # 10. 3D Kinematic Rotation Matrix Integration (Body Velocities -> World Frame Positions)
        cos_p = math.cos(pitch_rad)
        sin_p = math.sin(pitch_rad)
        cos_y = math.cos(yaw_rad)
        sin_y = math.sin(yaw_rad)
        cos_r = math.cos(roll_rad)
        sin_r = math.sin(roll_rad)

        dx_dt = (self.u * cos_p * cos_y +
                 self.v * (sin_r * sin_p * cos_y - cos_r * sin_y) +
                 self.w * (cos_r * sin_p * cos_y + sin_r * sin_y))

        dy_dt = (self.u * cos_p * sin_y +
                 self.v * (sin_r * sin_p * sin_y + cos_r * cos_y) +
                 self.w * (cos_r * sin_p * sin_y - sin_r * cos_y))

        dz_dt = (- self.u * sin_p +
                 self.v * sin_r * cos_p +
                 self.w * cos_r * cos_p)

        self.x += dx_dt * dt
        self.y += dy_dt * dt
        self.z += dz_dt * dt

        self.vz = dz_dt  # Track world vertical rate

        # 11. Euler Angular Rate Integration
        p_deg = math.degrees(self.p)
        q_deg = math.degrees(self.q)
        r_deg = math.degrees(self.r)

        cos_pitch = max(0.01, cos_p)
        droll_dt = p_deg + q_deg * sin_r * math.tan(pitch_rad) + r_deg * cos_r * math.tan(pitch_rad)
        dpitch_dt = q_deg * cos_r - r_deg * sin_r
        dyaw_dt = (q_deg * sin_r + r_deg * cos_r) / cos_pitch

        self.roll += droll_dt * dt
        self.pitch += dpitch_dt * dt
        self.yaw += dyaw_dt * dt

        # Wrap angles
        while self.yaw > 180.0: self.yaw -= 360.0
        while self.yaw < -180.0: self.yaw += 360.0

        # 12. PHYSICAL WATER SURFACE BOUNDARY (Z <= 0)
        if self.z <= 0.0:
            self.z = 0.0
            if self.w < 0.0:
                self.w = 0.0
            if self.vz < 0.0:
                self.vz = 0.0

        # 13. PHYSICS DEBUG OUTPUT AT ~2 Hz
        turn_radius = self.u / max(0.001, abs(self.r))
        if self.step_counter % 25 == 0:
            self.get_logger().info(
                f"\nPHYSICS DEBUG\n"
                f"Z DEPTH:          {self.z:.2f} m\n"
                f"WORLD VERT VEL:   {self.vz:.2f} m/s (dz/dt)\n"
                f"BODY SURGE U:     {self.u:.2f} m/s\n"
                f"BODY HEAVE W:     {self.w:.2f} m/s\n"
                f"MASS:             {self.mass:.2f} kg\n"
                f"DISPLACED VOLUME: {effective_volume:.4f} m3\n"
                f"WEIGHT:           +{F_weight:.1f} N\n"
                f"BUOYANCY:         {F_buoyancy:.1f} N\n"
                f"NET VERTICAL:     {F_net_buoyancy:.1f} N\n"
                f"VERT HEAVE DRAG:  {F_drag_z:.1f} N\n"
                f"THRUST FORCE:     {F_thrust:.1f} N\n"
                f"FORWARD DRAG:     {F_drag_x:.1f} N\n"
                f"PITCH:            {self.pitch:.1f} deg\n"
                f"PITCH RATE:       {q_deg:.2f} deg/s\n"
                f"YAW RATE:         {r_deg:.2f} deg/s\n"
                f"ROLL RATE:        {p_deg:.2f} deg/s\n"
                f"ELEVATOR MOMENT:  {M_elevator:.2f} Nm\n"
                f"RESTORING MOMENT: {M_restore:.2f} Nm\n"
                f"TURN RADIUS:      {turn_radius:.2f} m\n"
            )

        # Publish state (Odometry)
        state_msg = Odometry()
        state_msg.x = float(self.x)
        state_msg.y = float(self.y)
        state_msg.z = float(self.z)
        state_msg.roll = float(self.roll)
        state_msg.pitch = float(self.pitch)
        state_msg.yaw = float(self.yaw)
        state_msg.vx = float(self.u)      # Body surge
        state_msg.vy = float(dy_dt)       # World vy
        state_msg.vz = float(self.vz)     # World vz (dz/dt)

        self.state_pub.publish(state_msg)

        # Publish Physics Debug Telemetry
        physics_dict = {
            "buoyancy_force": float(F_net_buoyancy),
            "thrust_force": float(F_thrust),
            "forward_drag": float(F_drag_x),
            "vertical_drag": float(F_drag_z),
            "dynamic_pressure": float(q_dyn),
            "pitch_moment": float(M_elevator),
            "yaw_moment": float(N_rudder),
            "roll_moment": float(L_roll),
            "restoring_moment": float(M_restore),
            "body_u": float(self.u),
            "body_w": float(self.w),
            "vz_world": float(self.vz),
            "pitch_rate": float(q_deg),
            "yaw_rate": float(r_deg),
            "roll_rate": float(p_deg),
            "turn_radius": float(turn_radius),
            "weight": float(F_weight),
            "cg_position": [float(self.x), float(self.y), float(self.z)],
            "cb_position": [float(self.x), float(self.y), float(self.z - self.z_bg)]
        }
        phys_msg = String()
        phys_msg.data = json.dumps(physics_dict)
        self.physics_pub.publish(phys_msg)


def main(args=None):
    rclpy.init(args=args)
    node = IdealAUV()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
