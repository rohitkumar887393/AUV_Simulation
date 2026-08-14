#!/usr/bin/env python3

import math
import sys


class IdealAUVPhysicsModel:
    """Pure mathematical representation of IdealAUV physics step for unit testing."""

    def __init__(self, sim_rate=50.0):
        self.sim_rate = sim_rate
        self.tau_servo = 0.10
        self.tau_thruster = 0.20

        self.fwd_accel_gain = 1.0
        self.surge_damping = 0.4

        self.yaw_accel_gain = 10.0
        self.yaw_damping = 2.0

        self.pitch_accel_gain = 10.0
        self.pitch_damping = 2.0

        self.roll_accel_gain = 10.0
        self.roll_damping = 3.0

        self.max_fwd_vel = 3.0
        self.max_ang_vel = 45.0

        self.reset()

    def reset(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0

        self.roll_rate = 0.0
        self.pitch_rate = 0.0
        self.yaw_rate = 0.0

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

    def step(self, duration_sec=0.02):
        dt = duration_sec

        norm_cmd_el_l = (float(self.cmd_el_l) - 1500.0) / 400.0
        norm_cmd_el_r = (float(self.cmd_el_r) - 1500.0) / 400.0
        norm_cmd_rud_l = (float(self.cmd_rud_l) - 1500.0) / 400.0
        norm_cmd_rud_r = (float(self.cmd_rud_r) - 1500.0) / 400.0
        norm_cmd_thruster = (float(self.cmd_thruster) - 1500.0) / 400.0

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

        # Surge
        accel_surge = (self.act_thruster * self.fwd_accel_gain) - (self.surge_damping * self.vx)
        self.vx += accel_surge * dt
        self.vx = max(-self.max_fwd_vel, min(self.max_fwd_vel, self.vx))

        # Yaw
        rudder_input = (self.act_rud_l - self.act_rud_r) / 2.0
        accel_yaw = (rudder_input * self.yaw_accel_gain) - (self.yaw_damping * self.yaw_rate)
        self.yaw_rate += accel_yaw * dt
        self.yaw_rate = max(-self.max_ang_vel, min(self.max_ang_vel, self.yaw_rate))
        self.yaw += self.yaw_rate * dt

        while self.yaw > 180.0: self.yaw -= 360.0
        while self.yaw < -180.0: self.yaw += 360.0

        # Pitch
        pitch_input = (self.act_el_l - self.act_el_r) / 2.0
        accel_pitch = (pitch_input * self.pitch_accel_gain) - (self.pitch_damping * self.pitch_rate)
        self.pitch_rate += accel_pitch * dt
        self.pitch_rate = max(-self.max_ang_vel, min(self.max_ang_vel, self.pitch_rate))
        self.pitch += self.pitch_rate * dt
        self.pitch = max(-85.0, min(85.0, self.pitch))

        # Roll
        roll_input = (self.act_el_l + self.act_el_r) / 2.0
        accel_roll = (roll_input * self.roll_accel_gain) - (self.roll_damping * self.roll_rate)
        self.roll_rate += accel_roll * dt
        self.roll_rate = max(-self.max_ang_vel, min(self.max_ang_vel, self.roll_rate))
        self.roll += self.roll_rate * dt
        self.roll = max(-45.0, min(45.0, self.roll))

        # Kinematics
        yaw_rad = math.radians(self.yaw)
        pitch_rad = math.radians(self.pitch)

        vx_world = self.vx * math.cos(pitch_rad) * math.cos(yaw_rad)
        vy_world = self.vx * math.cos(pitch_rad) * math.sin(yaw_rad)
        vz_world = - self.vx * math.sin(pitch_rad)

        self.vy = vy_world
        self.vz = vz_world

        self.x += vx_world * dt
        self.y += vy_world * dt
        self.z += vz_world * dt
        self.z = max(0.0, self.z)


def run_tests():
    print("=" * 60)
    print("RUNNING AUTOMATED AUV PHYSICS VERIFICATION SUITE")
    print("=" * 60)

    model = IdealAUVPhysicsModel()
    all_passed = True

    # TEST A — Neutral PWM = 1500
    model.reset()
    model.cmd_thruster = 1500
    for _ in range(100):  # 2 seconds
        model.step()

    test_a_pass = abs(model.vx) < 1e-3 and abs(model.x) < 1e-3
    print(f"[TEST A] Neutral PWM=1500 -> Accel ≈ 0, Vel = {model.vx:.4f} m/s, Pos = {model.x:.4f} m : {'PASS' if test_a_pass else 'FAIL'}")
    if not test_a_pass: all_passed = False

    # TEST B — Forward Acceleration PWM = 1800
    model.reset()
    model.cmd_thruster = 1800
    for _ in range(100):  # 2 seconds forward thrust
        model.step()

    test_b_pass = model.vx > 0.5 and model.x > 0.5
    print(f"[TEST B] Forward Acceleration PWM=1800 -> Vel = {model.vx:.4f} m/s, Pos = {model.x:.4f} m : {'PASS' if test_b_pass else 'FAIL'}")
    if not test_b_pass: all_passed = False

    # TEST C — Stop / Coast PWM = 1500 after 1800
    # Continue from TEST B
    start_x = model.x
    start_vx = model.vx
    model.cmd_thruster = 1500

    # Wait for thruster spool down (10 steps = 0.2s)
    for _ in range(10):
        model.step()

    peak_vx = model.vx
    prev_vx = peak_vx
    is_decaying = True
    for _ in range(150):
        model.step()
        if model.vx > prev_vx + 1e-5:
            is_decaying = False
        prev_vx = model.vx

    stopping_distance = model.x - start_x
    test_c_pass = is_decaying and model.vx < peak_vx * 0.5 and stopping_distance > 0.5
    print(f"[TEST C] Coasting PWM=1500 -> Initial Vel = {start_vx:.2f} m/s, Final Vel = {model.vx:.4f} m/s, Stopping Distance = {stopping_distance:.2f} m : {'PASS' if test_c_pass else 'FAIL'}")
    if not test_c_pass: all_passed = False

    # TEST D — Reverse PWM = 1200
    model.reset()
    model.cmd_thruster = 1200
    for _ in range(100):
        model.step()

    test_d_pass = model.vx < -0.3 and model.x < -0.3
    print(f"[TEST D] Reverse PWM=1200 -> Vel = {model.vx:.4f} m/s, Pos = {model.x:.4f} m : {'PASS' if test_d_pass else 'FAIL'}")
    if not test_d_pass: all_passed = False

    # TEST E — Heading (Yaw = 0° vs 90°)
    # Yaw = 0°
    model.reset()
    model.yaw = 0.0
    model.cmd_thruster = 1800
    for _ in range(100): model.step()
    x_yaw0 = model.x
    y_yaw0 = model.y

    # Yaw = 90°
    model.reset()
    model.yaw = 90.0
    model.cmd_thruster = 1800
    for _ in range(100): model.step()
    x_yaw90 = model.x
    y_yaw90 = model.y

    test_e_pass = (x_yaw0 > 0.5 and abs(y_yaw0) < 1e-3) and (abs(x_yaw90) < 1e-3 and y_yaw90 > 0.5)
    print(f"[TEST E] Heading Yaw=0° vs 90° -> Yaw 0° Pos=({x_yaw0:.2f}, {y_yaw0:.2f}), Yaw 90° Pos=({x_yaw90:.2f}, {y_yaw90:.2f}) : {'PASS' if test_e_pass else 'FAIL'}")
    if not test_e_pass: all_passed = False

    # TEST F — Pitch Down (Elevator Pitching Down -> Depth Increase)
    model.reset()
    model.pitch = -15.0  # 15 degrees nose down
    model.cmd_thruster = 1800
    for _ in range(100): model.step()

    test_f_pass = model.z > 0.1 and model.vz > 0.05
    print(f"[TEST F] Pitch Down -15° -> Depth = {model.z:.2f} m, Vz = {model.vz:.2f} m/s : {'PASS' if test_f_pass else 'FAIL'}")
    if not test_f_pass: all_passed = False

    # TEST G — Level Vehicle (Pitch = 0° -> No Depth Change)
    model.reset()
    model.pitch = 0.0
    model.cmd_thruster = 1800
    for _ in range(100): model.step()

    test_g_pass = abs(model.z) < 1e-3 and abs(model.vz) < 1e-3
    print(f"[TEST G] Level Vehicle Pitch=0° -> Depth = {model.z:.4f} m, Vz = {model.vz:.4f} m/s : {'PASS' if test_g_pass else 'FAIL'}")
    if not test_g_pass: all_passed = False

    print("=" * 60)
    if all_passed:
        print("ALL PHYSICS TESTS PASSED PERFECTLY!")
        print("=" * 60)
        sys.exit(0)
    else:
        print("SOME PHYSICS TESTS FAILED!")
        print("=" * 60)
        sys.exit(1)


if __name__ == '__main__':
    run_tests()
