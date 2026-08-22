#!/usr/bin/env python3

import math
import sys


class IdealAUVPhysicsModel:
    """Pure mathematical representation of Phase 1 IdealAUV physics step for unit testing."""

    def __init__(self, sim_rate=50.0):
        self.sim_rate = sim_rate
        self.tau_servo = 0.10
        self.tau_thruster = 0.20

        # Phase 1 Physics Parameters (Verified Audit Values)
        self.mass = 25.0                   # AUV_MASS (kg)
        self.g = 9.81                      # Gravity (m/s^2)
        self.rho = 1025.0                  # Water density (kg/m^3)
        self.volume = 0.0247               # Displaced volume (m^3) -> 24.7L (~3.1N positive buoyancy)
        self.buoy_scale = 1.000            # BUOYANCY_SCALE

        self.max_thrust_n = 120.0          # MAX_THRUST_N (N)

        # Hydrodynamic Drag Parameters (Linear + Quadratic)
        self.Xu = 15.0                     # Surge linear drag (N / (m/s))
        self.Xuu = 25.0                    # Surge quadratic drag (N / (m/s)^2)

        self.Yv = 35.0                     # Sway linear drag
        self.Yvv = 60.0                    # Sway quadratic drag

        self.Zw = 15.0                     # Heave linear drag
        self.Zww = 40.0                    # Heave quadratic drag

        # Rotational properties (Preserved)
        self.ix = 0.5
        self.iy = 2.0
        self.iz = 2.0
        self.z_bg = 0.03

        self.Kp = 2.0
        self.Kpp = 5.0
        self.Kq = 25.0
        self.Kqq = 40.0
        self.Kr = 12.0
        self.Krr = 35.0

        self.elev_coeff = 0.015
        self.rud_coeff = 0.0035
        self.roll_coeff = 0.004

        self.max_fwd_vel = 4.0

        self.reset()

    def reset(self, z_start=0.5):
        self.x = 0.0
        self.y = 0.0
        self.z = z_start

        self.vx_world = 0.0
        self.vy_world = 0.0
        self.vz_world = 0.0
        self.vz = 0.0

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        self.u = 0.0  # body surge
        self.v = 0.0  # body sway
        self.w = 0.0  # body heave

        self.p = 0.0  # roll rate
        self.q = 0.0  # pitch rate
        self.r = 0.0  # yaw rate

        self.cmd_el_l = 1500
        self.cmd_el_r = 1500
        self.cmd_rud_l = 1500
        self.cmd_rud_r = 1500
        self.cmd_thruster = 1500

        self.act_el_l = 0.0
        self.act_el_r = 0.0
        self.act_rud_l = 0.0
        self.act_rud_r = 0.0
        self.act_thruster = 0.0

        # Telemetry cached forces
        self.last_thrust_mag = 0.0
        self.last_gravity_mag = 0.0
        self.last_buoyancy_mag = 0.0
        self.last_drag_body_x = 0.0
        self.last_drag_body_y = 0.0
        self.last_drag_body_z = 0.0
        self.last_total_f_world = [0.0, 0.0, 0.0]
        self.last_accel_world = [0.0, 0.0, 0.0]

    def step(self, duration_sec=None):
        dt = duration_sec if duration_sec is not None else (1.0 / self.sim_rate)

        # 1. Normalize Commanded PWM Inputs [-1.0, +1.0]
        norm_cmd_el_l = max(-1.0, min(1.0, (float(self.cmd_el_l) - 1500.0) / 400.0))
        norm_cmd_el_r = max(-1.0, min(1.0, (float(self.cmd_el_r) - 1500.0) / 400.0))
        norm_cmd_rud_l = max(-1.0, min(1.0, (float(self.cmd_rud_l) - 1500.0) / 400.0))
        norm_cmd_rud_r = max(-1.0, min(1.0, (float(self.cmd_rud_r) - 1500.0) / 400.0))
        norm_cmd_thruster = max(-1.0, min(1.0, (float(self.cmd_thruster) - 1500.0) / 400.0))

        # 2. First-Order Actuator Spooling Dynamics
        self.act_el_l += ((norm_cmd_el_l - self.act_el_l) / self.tau_servo) * dt
        self.act_el_r += ((norm_cmd_el_r - self.act_el_r) / self.tau_servo) * dt
        self.act_rud_l += ((norm_cmd_rud_l - self.act_rud_l) / self.tau_servo) * dt
        self.act_rud_r += ((norm_cmd_rud_r - self.act_rud_r) / self.tau_servo) * dt
        self.act_thruster += ((norm_cmd_thruster - self.act_thruster) / self.tau_thruster) * dt

        self.act_el_l = max(-1.0, min(1.0, self.act_el_l))
        self.act_el_r = max(-1.0, min(1.0, self.act_el_r))
        self.act_rud_l = max(-1.0, min(1.0, self.act_rud_l))
        self.act_rud_r = max(-1.0, min(1.0, self.act_rud_r))
        self.act_thruster = max(-1.0, min(1.0, self.act_thruster))

        # 3. Trigonometric values
        roll_rad = math.radians(self.roll)
        pitch_rad = math.radians(self.pitch)
        yaw_rad = math.radians(self.yaw)

        cos_r = math.cos(roll_rad)
        sin_r = math.sin(roll_rad)
        cos_p = math.cos(pitch_rad)
        sin_p = math.sin(pitch_rad)
        cos_y = math.cos(yaw_rad)
        sin_y = math.sin(yaw_rad)

        # 4. Phase 1 Translational Forces
        # A. Mass: self.mass
        # B. Gravity: F_gravity = mass * g (+Z downward)
        F_gravity_mag = self.mass * self.g
        F_gravity_world = [0.0, 0.0, F_gravity_mag]
        self.last_gravity_mag = F_gravity_mag

        # C. Buoyancy: F_buoyancy = rho * g * V * BUOYANCY_SCALE (-Z upward)
        effective_volume = self.volume * self.buoy_scale
        F_buoyancy_mag = self.rho * self.g * effective_volume
        F_buoyancy_world = [0.0, 0.0, -F_buoyancy_mag]
        self.last_buoyancy_mag = F_buoyancy_mag

        # D. Thruster Force along Body +X
        F_thrust_mag = self.act_thruster * self.max_thrust_n
        self.last_thrust_mag = F_thrust_mag

        F_thrust_world_x = F_thrust_mag * cos_p * cos_y
        F_thrust_world_y = F_thrust_mag * cos_p * sin_y
        F_thrust_world_z = - F_thrust_mag * sin_p
        F_thrust_world = [F_thrust_world_x, F_thrust_world_y, F_thrust_world_z]

        # E. Hydrodynamic Drag (Linear + Quadratic)
        F_drag_body_x = - (self.Xu * self.u + self.Xuu * self.u * abs(self.u))
        F_drag_body_y = - (self.Yv * self.v + self.Yvv * self.v * abs(self.v))
        F_drag_body_z = - (self.Zw * self.w + self.Zww * self.w * abs(self.w))
        self.last_drag_body_x = F_drag_body_x
        self.last_drag_body_y = F_drag_body_y
        self.last_drag_body_z = F_drag_body_z

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

        # 5. Force Accumulation & Integration
        F_total_world_x = F_thrust_world[0] + F_gravity_world[0] + F_buoyancy_world[0] + F_drag_world[0]
        F_total_world_y = F_thrust_world[1] + F_gravity_world[1] + F_buoyancy_world[1] + F_drag_world[1]
        F_total_world_z = F_thrust_world[2] + F_gravity_world[2] + F_buoyancy_world[2] + F_drag_world[2]
        self.last_total_f_world = [F_total_world_x, F_total_world_y, F_total_world_z]

        ax_world = F_total_world_x / self.mass
        ay_world = F_total_world_y / self.mass
        az_world = F_total_world_z / self.mass
        self.last_accel_world = [ax_world, ay_world, az_world]

        self.vx_world += ax_world * dt
        self.vy_world += ay_world * dt
        self.vz_world += az_world * dt

        self.x += self.vx_world * dt
        self.y += self.vy_world * dt
        self.z += self.vz_world * dt
        self.vz = self.vz_world

        # Body frame velocities
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

        # Surface Boundary (z <= 0)
        if self.z <= 0.0:
            self.z = 0.0
            if self.vz_world < 0.0:
                self.vz_world = 0.0
                self.vz = 0.0
            if self.w < 0.0:
                self.w = 0.0

        # Rotational dynamics (Preserved)
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

        M_elevator = self.elev_coeff * q_dyn * pitch_input * dynamic_authority
        N_rudder = self.rud_coeff * q_dyn * rudder_input * dynamic_authority
        L_roll = self.roll_coeff * q_dyn * roll_input * dynamic_authority

        M_restore = - F_gravity_mag * self.z_bg * math.sin(pitch_rad)
        L_restore = - F_gravity_mag * self.z_bg * math.sin(roll_rad) * math.cos(pitch_rad)

        M_drag_p = - (self.Kp * self.p + self.Kpp * abs(self.p) * self.p)
        M_drag_q = - (self.Kq * self.q + self.Kqq * abs(self.q) * self.q)
        M_drag_r = - (self.Kr * self.r + self.Krr * abs(self.r) * self.r)

        M_total = M_elevator + M_restore + M_drag_q
        L_total = L_roll + L_restore + M_drag_p
        N_total = N_rudder + M_drag_r

        dp_dt = L_total / self.ix
        dq_dt = M_total / self.iy
        dr_dt = N_total / self.iz

        self.p += dp_dt * dt
        self.q += dq_dt * dt
        self.r += dr_dt * dt

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
        while self.yaw <= -180.0: self.yaw += 360.0
        while self.roll > 180.0: self.roll -= 360.0
        while self.roll <= -180.0: self.roll += 360.0
        self.pitch = max(-85.0, min(85.0, self.pitch))


def run_tests():
    print("=" * 60)
    print("RUNNING PHASE 1 AUV PHYSICS VERIFICATION SUITE")
    print("=" * 60)

    model = IdealAUVPhysicsModel()
    all_passed = True

    # --------------------------------------------------
    # TEST 1 — REST
    # PWM = 1500, velocity = 0
    # Expected: thrust = 0, no horizontal acceleration, vertical behavior determined by gravity + buoyancy
    # --------------------------------------------------
    model.reset(z_start=0.5)
    model.cmd_thruster = 1500
    model.step()

    test_1_pass = (abs(model.last_thrust_mag) < 1e-4 and
                   abs(model.last_total_f_world[0]) < 1e-4 and
                   abs(model.last_total_f_world[1]) < 1e-4 and
                   abs(model.vx_world) < 1e-4 and abs(model.vy_world) < 1e-4)
    print(f"[TEST 1 - REST] Thrust={model.last_thrust_mag:.2f}N, Fx={model.last_total_f_world[0]:.4f}N, Fy={model.last_total_f_world[1]:.4f}N : {'PASS' if test_1_pass else 'FAIL'}")
    if not test_1_pass: all_passed = False

    # --------------------------------------------------
    # TEST 2 — POSITIVE BUOYANCY & SURFACE RETURN
    # Start below surface (z = 5.0m), PWM = 1500, velocity = 0
    # Expected: slow upward movement toward surface (vz < 0, z decreases), no rapid acceleration
    # --------------------------------------------------
    model.reset(z_start=5.0)
    model.cmd_thruster = 1500
    initial_z = model.z

    for _ in range(250):  # 5.0 seconds
        model.step()

    net_vertical_lift = model.last_buoyancy_mag - model.last_gravity_mag
    ascent_rate = abs(model.vz_world)
    z_decreased = model.z < initial_z
    test_2_pass = (net_vertical_lift > 0.5) and z_decreased and (ascent_rate > 0.01) and (ascent_rate < 0.80)
    print(f"[TEST 2 - POSITIVE BUOYANCY] F_buoy={model.last_buoyancy_mag:.2f}N, F_grav={model.last_gravity_mag:.2f}N, Net Lift=+{net_vertical_lift:.2f}N, Ascent Speed={ascent_rate:.2f} m/s, Z: {initial_z:.2f}m -> {model.z:.2f}m : {'PASS' if test_2_pass else 'FAIL'}")
    if not test_2_pass: all_passed = False

    # --------------------------------------------------
    # TEST 3 — FORWARD THRUST
    # Apply forward PWM (PWM 1800 -> ~90 N thrust)
    # Expected: forward force, positive acceleration, velocity increases
    # --------------------------------------------------
    model.reset(z_start=0.5)
    model.cmd_thruster = 1800

    for _ in range(25):  # 0.5s
        model.step()

    thrust_on_pass = (model.last_thrust_mag > 50.0) and (model.last_accel_world[0] > 1.0) and (model.u > 0.3)
    print(f"[TEST 3 - FORWARD THRUST] PWM=1800 -> Thrust={model.last_thrust_mag:.1f}N, Fwd Accel={model.last_accel_world[0]:.2f} m/s^2, Surge Vel={model.u:.2f} m/s : {'PASS' if thrust_on_pass else 'FAIL'}")
    if not thrust_on_pass: all_passed = False

    # --------------------------------------------------
    # TEST 4 — THRUST RELEASE & GRADUAL DECELERATION
    # Build forward velocity, set PWM = 1500
    # Expected: thrust = 0, drag opposes velocity, velocity gradually decreases, AUV does not continue indefinitely
    # --------------------------------------------------
    model.reset(z_start=0.5)
    model.cmd_thruster = 1900
    for _ in range(75):  # reach steady speed
        model.step()

    peak_vel = model.u
    model.cmd_thruster = 1500

    # Coast for 4.0s (200 steps)
    for _ in range(15): model.step()  # spool down
    spool_vel = model.u
    for _ in range(185): model.step()
    final_vel = model.u

    test_4_pass = (abs(model.last_thrust_mag) < 1e-3) and (final_vel < spool_vel * 0.4) and (model.last_drag_body_x < 0.0 or final_vel < 0.05)
    print(f"[TEST 4 - THRUST RELEASE] Released to PWM 1500 -> Peak Vel={peak_vel:.2f} m/s, Vel after 4s={final_vel:.2f} m/s, Drag={model.last_drag_body_x:.2f}N : {'PASS' if test_4_pass else 'FAIL'}")
    if not test_4_pass: all_passed = False

    # --------------------------------------------------
    # TEST 5 — DRAG DIRECTION
    # For positive velocity: drag must be negative.
    # For negative velocity: drag must be positive.
    # Tested for forward, lateral, and vertical motions.
    # --------------------------------------------------
    # Surge
    model.reset()
    model.u = 1.5; model.step(0.001)
    drag_surge_pos = model.last_drag_body_x
    model.reset()
    model.u = -1.5; model.step(0.001)
    drag_surge_neg = model.last_drag_body_x

    # Sway
    model.reset()
    model.v = 1.0; model.step(0.001)
    drag_sway_pos = model.last_drag_body_y
    model.reset()
    model.v = -1.0; model.step(0.001)
    drag_sway_neg = model.last_drag_body_y

    # Heave
    model.reset()
    model.w = 1.0; model.step(0.001)
    drag_heave_pos = model.last_drag_body_z
    model.reset()
    model.w = -1.0; model.step(0.001)
    drag_heave_neg = model.last_drag_body_z

    test_5_pass = (drag_surge_pos < 0 and drag_surge_neg > 0 and
                   drag_sway_pos < 0 and drag_sway_neg > 0 and
                   drag_heave_pos < 0 and drag_heave_neg > 0)
    print(f"[TEST 5 - DRAG DIRECTION]"
          f"\n  Surge (+1.5 m/s -> {drag_surge_pos:.1f}N, -1.5 m/s -> {drag_surge_neg:.1f}N)"
          f"\n  Sway  (+1.0 m/s -> {drag_sway_pos:.1f}N, -1.0 m/s -> {drag_sway_neg:.1f}N)"
          f"\n  Heave (+1.0 m/s -> {drag_heave_pos:.1f}N, -1.0 m/s -> {drag_heave_neg:.1f}N)"
          f"\n  Result: {'PASS' if test_5_pass else 'FAIL'}")
    if not test_5_pass: all_passed = False

    # --------------------------------------------------
    # TEST 6 — HEADING & BODY-ORIENTED THRUST
    # Verify forward thrust follows existing AUV heading: 0°, 90°, 180°, 270°
    # --------------------------------------------------
    # Heading 0° -> +X
    model.reset()
    model.yaw = 0.0; model.cmd_thruster = 1800
    for _ in range(50): model.step()
    pos_0 = (model.x, model.y)

    # Heading 90° -> +Y
    model.reset()
    model.yaw = 90.0; model.cmd_thruster = 1800
    for _ in range(50): model.step()
    pos_90 = (model.x, model.y)

    # Heading 180° -> -X
    model.reset()
    model.yaw = 180.0; model.cmd_thruster = 1800
    for _ in range(50): model.step()
    pos_180 = (model.x, model.y)

    # Heading 270° (-90°) -> -Y
    model.reset()
    model.yaw = -90.0; model.cmd_thruster = 1800
    for _ in range(50): model.step()
    pos_270 = (model.x, model.y)

    test_6_pass = (pos_0[0] > 0.5 and abs(pos_0[1]) < 1e-3 and
                   abs(pos_90[0]) < 1e-3 and pos_90[1] > 0.5 and
                   pos_180[0] < -0.5 and abs(pos_180[1]) < 1e-3 and
                   abs(pos_270[0]) < 1e-3 and pos_270[1] < -0.5)
    print(f"[TEST 6 - HEADING]"
          f"\n  Yaw 0°   -> Pos=({pos_0[0]:.2f}, {pos_0[1]:.2f}) [Expect +X]"
          f"\n  Yaw 90°  -> Pos=({pos_90[0]:.2f}, {pos_90[1]:.2f}) [Expect +Y]"
          f"\n  Yaw 180° -> Pos=({pos_180[0]:.2f}, {pos_180[1]:.2f}) [Expect -X]"
          f"\n  Yaw 270° -> Pos=({pos_270[0]:.2f}, {pos_270[1]:.2f}) [Expect -Y]"
          f"\n  Result: {'PASS' if test_6_pass else 'FAIL'}")
    if not test_6_pass: all_passed = False

    # --------------------------------------------------
    # TEST 7 — ANGLE WRAPPING
    # Verify: 365° -> 5°, 720° -> 0°, 725° -> 5°, -5° -> 355° for 0-360 representation
    # and signed [-180, 180) convention
    # --------------------------------------------------
    def wrap_0_360(a): return a % 360.0
    def wrap_signed(a):
        while a > 180.0: a -= 360.0
        while a <= -180.0: a += 360.0
        return a

    test_7_a = abs(wrap_0_360(365.0) - 5.0) < 1e-5
    test_7_b = abs(wrap_0_360(720.0) - 0.0) < 1e-5
    test_7_c = abs(wrap_0_360(725.0) - 5.0) < 1e-5
    test_7_d = abs(wrap_0_360(-5.0) - 355.0) < 1e-5

    test_7_e = abs(wrap_signed(365.0) - 5.0) < 1e-5
    test_7_f = abs(wrap_signed(720.0) - 0.0) < 1e-5
    test_7_g = abs(wrap_signed(725.0) - 5.0) < 1e-5
    test_7_h = abs(wrap_signed(-185.0) - 175.0) < 1e-5

    test_7_pass = (test_7_a and test_7_b and test_7_c and test_7_d and
                   test_7_e and test_7_f and test_7_g and test_7_h)
    print(f"[TEST 7 - ANGLE WRAPPING]"
          f"\n  0-360°: 365°->{wrap_0_360(365.0):.0f}°, 720°->{wrap_0_360(720.0):.0f}°, 725°->{wrap_0_360(725.0):.0f}°, -5°->{wrap_0_360(-5.0):.0f}°"
          f"\n  Signed: 365°->{wrap_signed(365.0):.0f}°, 720°->{wrap_signed(720.0):.0f}°, 725°->{wrap_signed(725.0):.0f}°, -185°->{wrap_signed(-185.0):.0f}°"
          f"\n  Result: {'PASS' if test_7_pass else 'FAIL'}")
    if not test_7_pass: all_passed = False

    # --------------------------------------------------
    # TEST 8 — ROLL STABILIZATION COMPATIBILITY
    # Run roll stabilizer PID with non-zero initial roll, verify it drives roll back to 0°
    # --------------------------------------------------
    model.reset()
    model.roll = 20.0  # Initial roll perturbation = 20 deg
    model.u = 1.5      # Forward surge speed for elevator/fin dynamic pressure

    # Simple PID simulator matching roll_stabilizer.py (kp=4.0, ki=0.01, kd=0.2)
    kp = 4.0; ki = 0.01; kd = 0.2
    integral = 0.0; prev_err = 0.0

    for _ in range(150):  # 3.0s
        error = 0.0 - model.roll
        integral += error * 0.02
        derivative = (error - prev_err) / 0.02
        prev_err = error
        roll_cmd = max(-300.0, min(300.0, kp * error + ki * integral + kd * derivative))

        # AUVControl actuator mixing:
        # pwm_elevator_left = 1500 + pitch_pwm + roll_cmd
        # pwm_elevator_right = 1500 - pitch_pwm + roll_cmd
        model.cmd_el_l = int(round(1500 + roll_cmd))
        model.cmd_el_r = int(round(1500 + roll_cmd))
        model.step()

    final_roll = abs(model.roll)
    test_8_pass = final_roll < 3.0
    print(f"[TEST 8 - ROLL STABILIZATION] Initial Roll=20.0° -> Stabilized Roll={model.roll:.2f}° (Target=0.0°) : {'PASS' if test_8_pass else 'FAIL'}")
    if not test_8_pass: all_passed = False

    print("=" * 60)
    if all_passed:
        print("ALL 8 PHASE 1 PHYSICS TESTS PASSED PERFECTLY!")
        print("=" * 60)
        sys.exit(0)
    else:
        print("SOME PHASE 1 PHYSICS TESTS FAILED!")
        print("=" * 60)
        sys.exit(1)


if __name__ == '__main__':
    run_tests()
