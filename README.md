# ARM3DOF – Modular 3-DOF Robotic Arm Simulation

This project provides a simulation of a 3 Degrees of Freedom (3-DOF) articulated robotic arm with a gripper, developed for ROS 2 Jazzy and Gazebo Harmonic. 
The model supports both dynamic Xacro-based loading and static SDF-based loading, enabling flexible development and testing workflows.

---

## 1. Project Overview

| Feature | Description |
|------|-------------|
| **Robot Model** | 3-DOF Articulated Arm (Yaw, Shoulder, Elbow) |
| **Description Formats** | Xacro (source) and SDF (generated) |
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


# Create a workspace
`mkdir -p ~/arm_ws/src`
`cd ~/arm_ws/src`

# Clone the repository
`git clone https://github.com/EsatSimsek/arm3dof.git`

# Build the package
`cd ~/arm_ws`
`colcon build --packages-select arm3dof --symlink-install`

# Source the workspace
`source install/setup.bash`

## 4. Launch Scenarios (Dual Method)

The project can be launched using two different approaches via modular launch files located in the `launch` directory. These approaches allow either dynamic processing of the robot description during runtime or static loading from a pre-generated model file.

### A) Xacro Method (Recommended – Dynamic Loading)

This method processes the `model.urdf.xacro` file at runtime using `robot_state_publisher`, ensuring that the latest robot description is always used. Any modification made to the Xacro files is immediately reflected in the simulation without requiring manual regeneration steps. This approach is ideal for development, iteration, and debugging.

## Run Command:

`ros2 launch arm3dof arm_xacro_run.launch.py`

Workflow:
The arm_xacro_run.launch.py file invokes arm_load_xacro.launch.py for processing and loading the robot description and arm_common.launch.py for initializing the Gazebo simulation environment and shared components.

### B) SDF Method (Static Loading)

This method spawns the robot directly from the model.sdf file without runtime processing. It provides faster startup times and a more deterministic configuration, which can be useful for testing and demonstrations. If any changes are made to the model.urdf.xacro file, the model.sdf file must be manually regenerated before using this method.

## Run Command:

`ros2 launch arm3dof arm_sdf_run.launch.py`

Workflow:
The arm_sdf_run.launch.py file invokes arm_load_sdf.launch.py for spawning the robot from the SDF file and arm_common.launch.py for setting up the simulation environment.

### 5. Notes

This project does not currently use ros2_control. Joint commands are sent directly via ROS 2 topics bridged to Gazebo using ros_gz_bridge. The modular launch structure allows easy extension to controllers, sensors, or additional degrees of freedom.

### 6. License

This project is licensed under the MIT License.

