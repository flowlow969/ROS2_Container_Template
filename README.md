# DeltaFlex — Compliant Delta Robot (ROS 2 Jazzy + Gazebo Harmonic)

ROS 2 Jazzy simulation stack for the [DeltaFlex](https://github.com/made-iit/deltaflex) monolithic 3D-printed compliant delta robot.

**Stack:** ROS 2 Jazzy · Gazebo Harmonic · `gz_ros2_control` · `ros2_control`

---

## Robot Overview

| Parameter | Value |
|---|---|
| Transmission ratio | 3.8:1 (S2M timing belt + pulley) |
| Motor | NEMA 17, 1.8°/step (200 steps/rev) |
| Workspace XY | ±50 mm |
| Workspace Z | ±60 mm |
| Repeatability | < 30 µm |
| Electronics | Arduino Uno R4 WiFi (RA4M1) + 3-Axis CNC Shield + 3× A4988 |

---

## Package Structure

```
src/
├── deltaflex_msgs/          # IKSolve.srv and FKSolve.srv interface definitions
├── deltaflex_description/   # URDF/Xacro robot model
├── deltaflex_kinematics/    # Analytical IK, numerical FK, and ROS2 service node
├── deltaflex_controllers/   # Cartesian-space interpolation controller node
└── deltaflex_bringup/       # Launch files, controller config, Gazebo world
```

### ROS 2 Topics and Services

| Name | Type | Description |
|---|---|---|
| `/deltaflex_position_controller/commands` | `std_msgs/Float64MultiArray` | Joint angle commands [θ1, θ2, θ3] in radians |
| `/joint_states` | `sensor_msgs/JointState` | Joint state feedback (virtual encoder) |
| `/delta/target_pose` | `geometry_msgs/Point` | Cartesian EE target [x, y, z] in metres |
| `/deltaflex/ik` | `deltaflex_msgs/IKSolve` | IK service: EE position → joint angles |
| `/deltaflex/fk` | `deltaflex_msgs/FKSolve` | FK service: joint angles → EE position |

---

## Prerequisites

Open this folder in VS Code and reopen in the **Jazzy devcontainer**:
`.devcontainer/moveit2_jazzy/devcontainer.json`

All commands below run **inside the devcontainer**.

---

## Build

```bash
# Install ROS dependencies declared in package.xml files
./setup.sh

# Build all packages
./build.sh

# Source the workspace
source install/setup.bash
```

---

## Gazebo Simulation

### Launch the full simulation

```bash
ros2 launch deltaflex_bringup sim.launch.py
```

Optional arguments:

```bash
# Headless (no Gazebo GUI window — useful in CI or SSH sessions)
ros2 launch deltaflex_bringup sim.launch.py gui:=false

# Also open RViz2 for joint state visualisation
ros2 launch deltaflex_bringup sim.launch.py with_rviz:=true

# Launch without the Cartesian controller node
ros2 launch deltaflex_bringup sim.launch.py with_cartesian:=false
```

### Verify controllers are active

```bash
ros2 control list_controllers
# Expected output:
# joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
# deltaflex_position_controller[forward_command_controller/ForwardCommandController] active
```

---

## Sending Commands

### Direct joint angle command (bypasses IK)

```bash
ros2 topic pub --once /deltaflex_position_controller/commands \
  std_msgs/msg/Float64MultiArray '{data: [0.3, 0.3, 0.3]}'
```

### Cartesian target (IK resolved automatically)

Z is positive-downward. Typical working range: `z ∈ [0.15, 0.35]`, XY radius ≤ 50 mm.

```bash
# Move to home position
ros2 topic pub --once /delta/target_pose geometry_msgs/msg/Point \
  '{x: 0.0, y: 0.0, z: 0.25}'

# Offset 20 mm in X
ros2 topic pub --once /delta/target_pose geometry_msgs/msg/Point \
  '{x: 0.02, y: 0.0, z: 0.25}'
```

### Call IK service directly

```bash
ros2 service call /deltaflex/ik deltaflex_msgs/srv/IKSolve \
  '{x: 0.0, y: 0.0, z: 0.25}'
```

### Call FK service directly

```bash
ros2 service call /deltaflex/fk deltaflex_msgs/srv/FKSolve \
  '{joint_angles: [0.1, 0.1, 0.1]}'
```

---

## Visualise URDF Only (no Gazebo)

```bash
ros2 launch deltaflex_description view_robot.launch.py
```

Opens `joint_state_publisher_gui` with sliders and RViz2 for model inspection.

---

## Run Tests

```bash
./test.sh
```

---

## Coordinate Convention

- Z is **positive downward** (robot hangs from a ceiling mount).
- Home position: EE directly below base centre at approximately `z = 0.25 m`.
- Joint angle `θ = 0` → upper arm horizontal.
- Positive θ → arm rotates downward.

---

## Kinematic Parameters

Defaults match the DeltaFlex paper (Parmiggiani et al.). Update `delta_ik.py` once STEP-file measurements are available.

| Symbol | Default | Description |
|---|---|---|
| `F` | 0.100 m | Base circumradius |
| `E` | 0.040 m | End-effector circumradius |
| `RF` | 0.120 m | Upper arm length |
| `RE` | 0.200 m | Forearm length |

---

## References

- [DeltaFlex repository](https://github.com/made-iit/deltaflex)
- [gz_ros2_control (Jazzy)](https://control.ros.org/jazzy/doc/gz_ros2_control/doc/index.html)
- [ros2_control controllers index](https://control.ros.org/jazzy/doc/ros2_controllers/doc/controllers_index.html)
- [Gazebo Harmonic + ROS 2 compatibility](https://gazebosim.org/docs/latest/ros_installation/)