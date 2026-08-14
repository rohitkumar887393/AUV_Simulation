# AUV Simulation

Simulation environment for testing the Autonomous Underwater Vehicle (AUV) control and autonomy software without requiring physical vehicle hardware.

The primary goal of this project is to provide a practical simulation environment for validating the real AUV's control logic, PID controllers, joystick operation, autonomous missions, actuator behavior, and vehicle response before deploying changes to the physical vehicle.

---

## Purpose

This simulation is intended for **control-logic and mission validation**, rather than high-fidelity CFD or a visually realistic underwater environment.

The simulator provides a virtual AUV with:

- 6-DOF vehicle motion
- Main thruster dynamics
- Elevator dynamics
- Rudder dynamics
- Buoyancy
- Gravity
- Linear and angular drag
- Vehicle inertia
- Center of Gravity / Center of Buoyancy effects
- Hydrodynamic control-surface behavior
- Actuator response dynamics
- Simulated sensor feedback

---

## Relationship with the Real AUV

The real AUV software and simulation software are maintained in separate repositories.

The **real AUV repository is the source of truth** for vehicle control and autonomy software.

The simulation repository is used to test the same control and autonomy logic against simulated vehicle dynamics.

```text
                 REAL AUV REPOSITORY
                         |
                         | Shared control,
                         | autonomy, navigation
                         | and interfaces
                         v
                 SIMULATION REPOSITORY
                         |
                         v
                  Simulation Physics
                         |
                         v
                    Virtual AUV
