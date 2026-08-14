# AUV Simulation & Digital Twin

A comprehensive ROS 2 workspace for Autonomous Underwater Vehicle (AUV) simulation, control, navigation, telemetry, and 3D visualization.

## ROS 2 Packages

- **`auv_interfaces`**: Custom ROS 2 messages for AUV sensors, actuators, missions, and telemetry (`ActuatorCmds`, `Battery`, `Depth`, `Leak`, `Mission`, `Odometry`, `Orientation`).
- **`auv_simulation`**: Physics simulation engine, actuator/sensor simulators, 2D/3D visualizers, and web visualizers.
- **`auv_autonomy`**: High-level mission management, PID control loops (depth hold, heading hold, distance hold, roll stabilization).
- **`auv_navigation`**: Odometry fusion and depth/pitch navigation controllers.
- **`auv_sensors`**: Drivers and ROS 2 nodes for IMU, DVL, Depth Sensor, Battery Monitor, and Leak Sensor.
- **`auv_teleop`**: Manual teleoperation and command receiver interfaces.
- **`auv_bringup`**: Launch files and configuration parameters for bringing up full simulation or hardware pipelines.
- **`auv_monitor`**: Real-time status monitoring and diagnostics.

## Getting Started

### Prerequisites

- ROS 2 Humble (Ubuntu 22.04 / WSL2)
- Colcon build tools

### Build

```bash
colcon build
```

### Usage

Source the workspace setup script:

```bash
source install/setup.bash
```

Launch 3D AUV Simulation:

```bash
ros2 launch auv_simulation simulation.launch.py enable_3d:=true
```
