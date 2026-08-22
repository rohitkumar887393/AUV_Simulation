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

        # Phase 1 Physics Parameters
        self.declare_parameter('mass', 25.0)                   # AUV_MASS (kg)
        self.declare_parameter('ix', 0.5)
        self.declare_parameter('iy', 2.0)
        self.declare_parameter('iz', 2.0)

        self.declare_parameter('water_density', 1025.0)        # rho_water (kg/m^3)
        self.declare_parameter('displaced_volume', 0.0247)     # AUV_VOLUME (m^3) -> 24.7 L gives ~3.1N net upward lift
        self.declare_parameter('gravity', 9.81)                # g (m/s^2)
        self.declare_parameter('buoyancy_scale', 1.000)        # BUOYANCY_SCALE
        self.declare_parameter('neutral_buoyancy_ratio', 1.000)# Backward-compatibility alias
        self.declare_parameter('z_bg', 0.03)

        self.declare_parameter('max_thrust_n', 120.0)          # MAX_THRUST_N (N)
        self.declare_parameter('thrust_coefficient', 120.0)    # Backward-compatibility alias

        # Hydrodynamic Drag Parameters (Linear + Quadratic)
        self.declare_parameter('linear_drag_x', 15.0)
        self.declare_parameter('quadratic_drag_x', 25.0)

        self.declare_parameter('linear_drag_y', 35.0)
        self.declare_parameter('quadratic_drag_y', 60.0)

        self.declare_parameter('linear_drag_z', 15.0)
        self.declare_parameter('quadratic_drag_z', 40.0)

        self.declare_parameter('cd_forward', 0.30)             # Cd_forward
        self.declare_parameter('cd_lateral', 1.00)             # Cd_lateral
        self.declare_parameter('cd_vertical', 1.00)            # Cd_vertical

        self.declare_parameter('area_forward', 0.03)           # A_forward (m^2)
        self.declare_parameter('area_lateral', 0.20)           # A_lateral (m^2)
        self.declare_parameter('area_vertical', 0.20)          # A_vertical (m^2)

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
        
        # Support buoyancy_scale or neutral_buoyancy_ratio
        if self.has_parameter('buoyancy_scale'):
            self.buoy_scale = float(self.get_parameter('buoyancy_scale').value)
        else:
            self.buoy_scale = float(self.get_parameter('neutral_buoyancy_ratio').value)
        self.z_bg = float(self.get_parameter('z_bg').value)

        if self.has_parameter('max_thrust_n'):
            self.max_thrust_n = float(self.get_parameter('max_thrust_n').value)
        else:
            self.max_thrust_n = float(self.get_parameter('thrust_coefficient').value)

        self.cd_fwd = float(self.get_parameter('cd_forward').value)
        self.cd_lat = float(self.get_parameter('cd_lateral').value)
        self.cd_vert = float(self.get_parameter('cd_vertical').value)

        self.area_fwd = float(self.get_parameter('area_forward').value)
        self.area_lat = float(self.get_parameter('area_lateral').value)
        self.area_vert = float(self.get_parameter('area_vertical').value)

        self.Xu = float(self.get_parameter('linear_drag_x').value)
        self.Yv = float(self.get_parameter('linear_drag_y').value)
        self.Zw = float(self.get_parameter('linear_drag_z').value)

        if self.has_parameter('quadratic_drag_x') and float(self.get_parameter('quadratic_drag_x').value) > 0.0:
            self.Xuu = float(self.get_parameter('quadratic_drag_x').value)
        else:
            self.Xuu = 0.5 * self.rho * self.cd_fwd * self.area_fwd

        if self.has_parameter('quadratic_drag_y') and float(self.get_parameter('quadratic_drag_y').value) > 0.0:
            self.Yvv = float(self.get_parameter('quadratic_drag_y').value)
        else:
            self.Yvv = 0.5 * self.rho * self.cd_lat * self.area_lat

        if self.has_parameter('quadratic_drag_z') and float(self.get_parameter('quadratic_drag_z').value) > 0.0:
            self.Zww = float(self.get_parameter('quadratic_drag_z').value)
        else:
            self.Zww = 0.5 * self.rho * self.cd_vert * self.area_vert

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

        effective_volume = self.volume * self.buoy_scale
        F_weight = self.mass * self.g
        F_buoyancy = self.rho * self.g * effective_volume
        net_vertical = F_weight - F_buoyancy

        self.get_logger().info("==================================================")
        self.get_logger().info("PHASE 1 BASIC AUV PHYSICS CORE INITIALIZATION")
        self.get_logger().info("==================================================")
        self.get_logger().info(f"MASS (AUV_MASS):        {self.mass:.2f} kg")
        self.get_logger().info(f"DISPLACED VOLUME:       {effective_volume:.4f} m^3")
        self.get_logger().info(f"WATER DENSITY:          {self.rho:.1f} kg/m^3")
        self.get_logger().info(f"GRAVITY:                {self.g:.2f} m/s^2")
        self.get_logger().info(f"GRAVITY FORCE (WEIGHT): +{F_weight:.2f} N (World +Z)")
        self.get_logger().info(f"BUOYANCY FORCE:         -{F_buoyancy:.2f} N (World -Z)")
        self.get_logger().info(f"NET VERTICAL FORCE:     {net_vertical:+.2f} N")
        self.get_logger().info(f"MAX THRUST (FORWARD):   {self.max_thrust_n:.1f} N")
        self.get_logger().info("==================================================")

    def reset_state(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.5  # Operational start depth underwater (0.5m)
        self.vx_world = 0.0
        self.vy_world = 0.0
        self.vz_world = 0.0
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
        response.message = "Phase 1 AUV simulation state and actuators reset to operational underwater state (z=0.5m)"
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

        # 1. Normalize Commanded PWM Inputs [-1.0, +1.0] (Neutral = 1500)
        norm_cmd_el_l = max(-1.0, min(1.0, (float(self.cmd_elevator_left) - 1500.0) / 400.0))
        norm_cmd_el_r = max(-1.0, min(1.0, (float(self.cmd_elevator_right) - 1500.0) / 400.0))
        norm_cmd_rud_l = max(-1.0, min(1.0, (float(self.cmd_rudder_left) - 1500.0) / 400.0))
        norm_cmd_rud_r = max(-1.0, min(1.0, (float(self.cmd_rudder_right) - 1500.0) / 400.0))
        norm_cmd_thruster = max(-1.0, min(1.0, (float(self.cmd_main_thruster) - 1500.0) / 400.0))

        # 2. Actuator Response Dynamics (Spooling)
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

        # 3. Trigonometric values & Orientation angles
        roll_rad = math.radians(self.roll)
        pitch_rad = math.radians(self.pitch)
        yaw_rad = math.radians(self.yaw)

        cos_r = math.cos(roll_rad)
        sin_r = math.sin(roll_rad)
        cos_p = math.cos(pitch_rad)
        sin_p = math.sin(pitch_rad)
        cos_y = math.cos(yaw_rad)
        sin_y = math.sin(yaw_rad)

        # =========================================================================
        # PHASE 1 TRANSLATIONAL PHYSICS (Forces -> Acceleration -> Velocity -> Pos)
        # =========================================================================

        # A. MASS
        # self.mass (kg)

        # B. GRAVITY (World +Z Downward)
        # F_gravity = mass * g
        F_gravity_mag = self.mass * self.g
        F_gravity_world = [0.0, 0.0, F_gravity_mag]

        # C. BUOYANCY (World -Z Upward)
        # F_buoyancy = rho_water * g * displaced_volume * BUOYANCY_SCALE
        effective_volume = self.volume * self.buoy_scale
        F_buoyancy_mag = self.rho * self.g * effective_volume
        F_buoyancy_world = [0.0, 0.0, -F_buoyancy_mag]

        # D. THRUSTER FORCE (Acts along Body Forward Axis +X_body)
        # thrust = normalized_command * MAX_THRUST_N
        F_thrust_mag = self.act_thruster * self.max_thrust_n
        # Transform body-frame thrust [F_thrust_mag, 0, 0] into world-frame using AUV orientation:
        F_thrust_world_x = F_thrust_mag * cos_p * cos_y
        F_thrust_world_y = F_thrust_mag * cos_p * sin_y
        F_thrust_world_z = - F_thrust_mag * sin_p
        F_thrust_world = [F_thrust_world_x, F_thrust_world_y, F_thrust_world_z]

        # E. HYDRODYNAMIC DRAG (Combined Linear + Quadratic: -C_lin*v - C_quad*v*|v|)
        F_drag_body_x = - (self.Xu * self.u + self.Xuu * self.u * abs(self.u))
        F_drag_body_y = - (self.Yv * self.v + self.Yvv * self.v * abs(self.v))
        F_drag_body_z = - (self.Zw * self.w + self.Zww * self.w * abs(self.w))

        # Transform body drag into world frame using R_B^W:
        F_drag_world_x = (F_drag_body_x * cos_p * cos_y +
                          F_drag_body_y * (sin_r * sin_p * cos_y - cos_r * sin_y) +
                          F_drag_body_z * (cos_r * sin_p * cos_y + sin_r * sin_y))

        F_drag_world_y = (F_drag_body_x * cos_p * sin_y +
                          F_drag_body_y * (sin_r * sin_p * sin_y + cos_r * cos_y) +
                          F_drag_body_z * (cos_r * sin_p * sin_y - sin_r * cos_y))

        F_drag_world_z = (- F_drag_body_x * sin_p +
                          F_drag_body_y * sin_r * cos_p +
                          F_drag_body_z * cos_r * cos_p)
        F_drag_world = [F_drag_world_x, F_drag_world_y, F_drag_world_z]

        # 8. FORCE ACCUMULATION (World Coordinates)
        # TOTAL_FORCE = THRUSTER_FORCE + BUOYANCY_FORCE + GRAVITY_FORCE + DRAG_FORCE
        F_total_world_x = F_thrust_world[0] + F_gravity_world[0] + F_buoyancy_world[0] + F_drag_world[0]
        F_total_world_y = F_thrust_world[1] + F_gravity_world[1] + F_buoyancy_world[1] + F_drag_world[1]
        F_total_world_z = F_thrust_world[2] + F_gravity_world[2] + F_buoyancy_world[2] + F_drag_world[2]

        # Translational Acceleration: a = TOTAL_FORCE / mass
        ax_world = F_total_world_x / self.mass
        ay_world = F_total_world_y / self.mass
        az_world = F_total_world_z / self.mass

        # Integration: velocity += acceleration * dt; position += velocity * dt
        self.vx_world += ax_world * dt
        self.vy_world += ay_world * dt
        self.vz_world += az_world * dt

        self.x += self.vx_world * dt
        self.y += self.vy_world * dt
        self.z += self.vz_world * dt

        self.vz = self.vz_world  # Track world vertical rate

        # Project world velocities back to body frame (u = surge, v = sway, w = heave) using (R_B^W)^T
        self.u = (self.vx_world * cos_p * cos_y +
                  self.vy_world * cos_p * sin_y -
                  self.vz_world * sin_p)

        self.v = (self.vx_world * (sin_r * sin_p * cos_y - cos_r * sin_y) +
                  self.vy_world * (sin_r * sin_p * sin_y + cos_r * cos_y) +
                  self.vz_world * sin_r * cos_p)

        self.w = (self.vx_world * (cos_r * sin_p * cos_y + sin_r * sin_y) +
                  self.vy_world * (cos_r * sin_p * sin_y - sin_r * cos_y) +
                  self.vz_world * cos_r * cos_p)

        self.u = max(-self.max_fwd_vel, min(self.max_fwd_vel, self.u))

        # PHYSICAL WATER SURFACE BOUNDARY (Z <= 0)
        if self.z <= 0.0:
            self.z = 0.0
            if self.vz_world < 0.0:
                self.vz_world = 0.0
                self.vz = 0.0
            if self.w < 0.0:
                self.w = 0.0

        # =========================================================================
        # ROTATIONAL DYNAMICS (Phase 2 untouched - preserving control surfaces & righting moments)
        # =========================================================================
        pitch_input = (self.act_el_l - self.act_el_r) / 2.0
        roll_input = (self.act_el_l + self.act_el_r) / 2.0
        rudder_input = (self.act_rud_l - self.act_rud_r) / 2.0

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

        # Control Surface Moments
        M_elevator = self.elev_coeff * q_dyn * pitch_input * dynamic_authority
        N_rudder = self.rud_coeff * q_dyn * rudder_input * dynamic_authority
        L_roll = self.roll_coeff * q_dyn * roll_input * dynamic_authority

        # Hydrostatic Restoring Righting Moments (CG/CB metacentric offset z_bg)
        M_restore = - F_gravity_mag * self.z_bg * math.sin(pitch_rad)
        L_restore = - F_gravity_mag * self.z_bg * math.sin(roll_rad) * math.cos(pitch_rad)

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

        # Euler Angular Rate Integration
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

        # Wrap angles to prevent indefinite growth
        while self.yaw > 180.0: self.yaw -= 360.0
        while self.yaw <= -180.0: self.yaw += 360.0
        while self.roll > 180.0: self.roll -= 360.0
        while self.roll <= -180.0: self.roll += 360.0
        self.pitch = max(-85.0, min(85.0, self.pitch))


        # =========================================================================
        # DEBUG TELEMETRY & LOGGING
        # =========================================================================
        net_vertical_force = F_total_world_z
        net_forward_force = F_thrust_mag + F_drag_body_x
        turn_radius = self.u / max(0.001, abs(self.r))

        if self.step_counter % 25 == 0:
            self.get_logger().info(
                f"\nPHASE 1 PHYSICS DEBUG\n"
                f"MASS:             {self.mass:.2f} kg\n"
                f"GRAVITY FORCE:    +{F_gravity_mag:.2f} N (World +Z)\n"
                f"BUOYANCY FORCE:   -{F_buoyancy_mag:.2f} N (World -Z)\n"
                f"NET VERTICAL F:   {net_vertical_force:+.2f} N\n"
                f"THRUST FORCE:     {F_thrust_mag:.2f} N (Body +X)\n"
                f"FORWARD DRAG:     {F_drag_body_x:.2f} N (Body -X)\n"
                f"NET FORWARD F:    {net_forward_force:+.2f} N\n"
                f"Z DEPTH:          {self.z:.2f} m\n"
                f"WORLD VERT VEL:   {self.vz_world:.2f} m/s\n"
                f"BODY SURGE U:     {self.u:.2f} m/s\n"
                f"PITCH:            {self.pitch:.1f} deg\n"
                f"YAW:              {self.yaw:.1f} deg\n"
            )

        # Publish state (Odometry)
        state_msg = Odometry()
        state_msg.x = float(self.x)
        state_msg.y = float(self.y)
        state_msg.z = float(self.z)
        state_msg.roll = float(self.roll)
        state_msg.pitch = float(self.pitch)
        state_msg.yaw = float(self.yaw)
        state_msg.vx = float(self.u)          # Body surge
        state_msg.vy = float(self.vy_world)   # World vy
        state_msg.vz = float(self.vz_world)   # World vz

        self.state_pub.publish(state_msg)

        # Publish Physics Debug Telemetry
        physics_dict = {
            "mass": float(self.mass),
            "gravity_force": float(F_gravity_mag),
            "buoyancy_force": float(-F_buoyancy_mag),
            "net_vertical_force": float(net_vertical_force),
            "thrust_force": float(F_thrust_mag),
            "thrust_world": [float(F_thrust_world[0]), float(F_thrust_world[1]), float(F_thrust_world[2])],
            "drag_body": [float(F_drag_body_x), float(F_drag_body_y), float(F_drag_body_z)],
            "drag_world": [float(F_drag_world[0]), float(F_drag_world[1]), float(F_drag_world[2])],
            "forward_drag": float(F_drag_body_x),
            "vertical_drag": float(F_drag_body_z),
            "net_forward_force": float(net_forward_force),
            "total_force_world": [float(F_total_world_x), float(F_total_world_y), float(F_total_world_z)],
            "accel_world": [float(ax_world), float(ay_world), float(az_world)],
            "dynamic_pressure": float(q_dyn),
            "pitch_moment": float(M_elevator),
            "yaw_moment": float(N_rudder),
            "roll_moment": float(L_roll),
            "restoring_moment": float(M_restore),
            "body_u": float(self.u),
            "body_v": float(self.v),
            "body_w": float(self.w),
            "vx_world": float(self.vx_world),
            "vy_world": float(self.vy_world),
            "vz_world": float(self.vz_world),
            "pitch_rate": float(q_deg),
            "yaw_rate": float(r_deg),
            "roll_rate": float(p_deg),
            "turn_radius": float(turn_radius),
            "weight": float(F_gravity_mag),
            "displaced_volume": float(effective_volume),
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

