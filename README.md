# 🌊 AUV Simulation & Control Stack (`AUV_sim`)

A high-fidelity **Software-in-the-Loop (SIL)** simulation and autonomy framework for Autonomous Underwater Vehicles (AUVs) running on **ROS 2 Humble**.

This repository hosts the full software stack—including autonomy controllers, teleoperation drivers, sensor bridges, and monitoring tools. It is engineered with strict **hardware-software isolation**, allowing you to develop, test, and validate mission algorithms in simulation and deploy the **exact same code** to physical AUV hardware without changing a single line of control logic.

## 🔑 Key Highlights

- **100% Code Parity (Sim-to-Real):** Autonomy nodes (`depth_hold`, `heading_hold`, `distance_hold`) run identically whether driven by virtual simulation topics or real physical sensor drivers / micro-ROS.
- **Hardware Abstraction Bridge (`sim_bridge`):** Mimics AUV kinematics, sensor responses (DVL, IMU, Pressure Depth, Leak), and MAVROS/Pixhawk telemetry topics (`/mavros/state`) when physical hardware is detached.
- **Deterministic Time Management:** Uses ROS 2 time abstraction (`self.get_clock().now()`) and `use_sim_time` to guarantee identical PID execution across both simulated time and wall-clock hardware time.
- **Dual Autonomy Modes:**
  - **Manual Assist / Single Toggles:** Quick joystick-driven holds (`Depth Hold`, `Heading Hold`, `Roll Stabilization`).
  - **Command-Driven Point Localization:** Autonomous multi-axis navigation triggered via ROS 2 `/auv/mission` topic commands.


## 📁 Repository Structure

```text
AUV/
├── auv_autonomy/      # Mission management & PID control loops (depth, heading, distance, roll)
├── auv_bringup/       # Launch files (autonomy.launch.py, sensors.launch.py, sim_master.launch.py)
├── auv_interfaces/    # Custom ROS 2 message contracts (Mission.msg, Odometry.msg, Depth.msg)
├── auv_monitor/       # System health diagnostics & telemetry dashboard
├── auv_navigation/    # DVL position integration & virtual kinematic sim_bridge.py
├── auv_sensors/       # Sensor abstraction nodes (DVL, IMU, Depth, Leak)
└── auv_teleop/        # Joystick control, gain adjustments, & manual overrides
