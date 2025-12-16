# ARM3DOF – Modular 3-DOF Robotic Arm Simulation

This project provides a simulation of a 3 Degrees of Freedom (3-DOF) articulated robotic arm with a gripper, developed for ROS 2 Jazzy and Gazebo Harmonic.
The robot model is defined exclusively using Xacro and is processed dynamically at runtime.

---

## 1. Project Overview

| Feature | Description |
|------|-------------|
| **Robot Model** | 3-DOF Articulated Arm (Yaw, Shoulder, Elbow) |
| **Description Format** | Xacro (URDF) |
| **Simulator** | Gazebo Harmonic (via `ros_gz_sim`) |
| **Control Interface** | ROS 2 topic-based joint position commands via `ros_gz_bridge` |
| **License** | MIT |

---

## 2. Prerequisites

Ensure the following components are installed on your system.

### System Requirements
- Ubuntu 22.04 or newer
- ROS 2 Jazzy
- Gazebo Harmonic

### Required ROS Packages
- `ros-jazzy-xacro`
- `ros-jazzy-robot-state-publisher`
- `ros-jazzy-ros-gz-sim`
- `ros-jazzy-ros-gz-bridge`

### Simulator
- Gazebo Harmonic

---

## 3. Installation and Build


### Create a workspace
```bash
mkdir -p ~/arm_ws/src
cd ~/arm_ws/src
```
### Clone the repository
```bash
git clone https://github.com/EsatSimsek/arm3dof.git
```

### Build the package
```bash
cd ~/arm_ws
colcon build --packages-select arm3dof --symlink-install
```

### Source the workspace
```bash
source install/setup.bash
```

---

## 4. Launch (Xacro-Based Runtime Loading)

This method processes the `model.urdf.xacro` file at runtime using `robot_state_publisher`, ensuring that the latest robot description is always used. Any modification made to the Xacro files is immediately reflected in the simulation without requiring manual regeneration steps. This approach is ideal for development, iteration, and debugging.

### Run Command:
```bash
ros2 launch arm3dof arm_xacro_run.launch.py
```

Workflow:
The arm_xacro_run.launch.py file processes the Xacro-based robot description, launches robot_state_publisher, starts the Gazebo Harmonic simulation environment, and spawns the robotic arm into the simulator within a single unified launch workflow.

---

## 5. Notes

This project does not currently use ros2_control. Joint commands are sent directly via ROS 2 topics bridged to Gazebo using ros_gz_bridge. The modular launch structure allows easy extension to controllers, sensors, or additional degrees of freedom.

---

## 6. License

This project is licensed under the MIT License.

