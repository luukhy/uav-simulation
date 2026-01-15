# ROS 2 Quadcopter Simulation and PID Control

## Project Overview
This repository contains a quadcopter simulation environment developed using **ROS 2 Jazzy** and **Gazebo Harmonic (GZ Sim 8.10.0)**. The project implements a custom flight control stack, focusing on stable attitude control, teleoperation integration, and real-time sensor telemetry.



## Technical Features
* **Attitude Control System:** Custom PID implementation for stabilized Pitch, Roll, and Yaw dynamics.
* **Teleoperation Stack:** Full integration with `teleop_twist_joy` using an Xbox controller for manual flight testing.
* **TODO: FPV Integration:** First-Person View (FPV) camera sensor integrated into the drone's SDF model with real-time streaming via ROS 2.
* **TODO: Visual Telemetry:** Comprehensive visualization of IMU data, transform (TF) trees, and camera feeds within RViz2.

## System Architecture
The software architecture follows a modular ROS 2 design:
* **flight_controller:** High-frequency node managing PID loops and joystick command interpretation.
* **uav_sim:** Environment management node responsible for Gazebo world configuration and entity spawning.
* **ros_gz_bridge:** Bidirectional communication layer between the ROS 2 transport and Gazebo's internal messaging system.



## PID Control and Tuning
The control system utilizes a Parallel PID architecture. To prevent system instability during aggressive maneuvering, the derivative term tracks the change in the state measurement rather than the error signal. This provides a natural damping effect without the risk of infinite spikes from instantaneous target changes.

### Tuned Gain Parameters
| Parameter | Value | Functional Objective |
| :--- | :--- | :--- |
| **Kp** | 100.0 | Proportional stiffness and error correction speed |
| **Ki** | 5.0 | Integral compensation for steady-state drift and weight bias |
| **Kd** | 30.0 | Derivative damping to mitigate oscillations and overshoot |



## Installation and Setup

### Prerequisites
* ROS 2 Jazzy installed on Ubuntu 24.04.
* Gazebo Harmonic (GZ Sim 8).
* `ros_gz` and `teleop_twist_joy` packages.

### Build Instructions
```bash
# Create and navigate to workspace
mkdir -p ~/drone_ws/src
cd ~/drone_ws

# Clone repository
git clone https://github.com/luukhy/uav-simulation.git

# Build packages
colcon build 
source install/setup.bash

# Launch the simulation
ros2 launch uav_sim ros2 launch uav_sim simulation.launch.py 
