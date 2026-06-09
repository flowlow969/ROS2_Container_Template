# DeltaFlex ROS2 Jazzy + Gazebo Harmonic Integration Guide

**Target robot:** [DeltaFlex](https://github.com/made-iit/deltaflex) — a monolithic 3D-printed compliant Delta robot  
**Stack:** ROS2 Jazzy · Gazebo Harmonic · `gz_ros2_control` · `ros2_control`  
**Audience:** Advanced robotics engineer familiar with ROS2, `ros2_control`, ODrive/stepper hardware, and C++/Python

---

## Table of Contents

1. [Overview & Strategy](#1-overview--strategy)
2. [ROS2 Package Architecture](#2-ros2-package-architecture)
3. [URDF / Xacro Model](#3-urdf--xacro-model)
4. [Delta Robot Kinematics (IK/FK)](#4-delta-robot-kinematics-ikfk)
5. [ros2_control Configuration](#5-ros2_control-configuration)
6. [Gazebo Simulation Setup](#6-gazebo-simulation-setup)
7. [Real Hardware — Stepper Control](#7-real-hardware--stepper-control)
8. [Cartesian Control Node](#8-cartesian-control-node)
9. [MoveIt2 Integration](#9-moveit2-integration)
10. [Launch File Overview](#10-launch-file-overview)
11. [Step-by-Step Getting Started](#11-step-by-step-getting-started)
12. [Key Challenges & Tips](#12-key-challenges--tips)

---

## 1. Overview & Strategy

### Why no standard `ros2_control` hardware interface exists for stepper motors

`ros2_control` was designed around servo-class actuators that expose continuous feedback (encoders, resolvers). Its hardware interface contract requires a `read()` call that populates state interfaces — typically `position`, `velocity`, `effort` — and a `write()` call that commands them. Stepper motors in their vanilla open-loop configuration break this contract in two ways:

1. **No state feedback.** There is no sensor to read; the controller has no knowledge of whether a step was missed.
2. **Discrete command space.** Steppers are commanded in integer step counts, not floating-point radians. The conversion is deterministic but requires microstepping configuration to be fixed at build time.

This means you must implement a **custom hardware interface** that uses the *virtual encoder* pattern: the state interface echoes back the last commanded position. This is honest about the open-loop nature of the system while satisfying the `ros2_control` type contract.

### Two-layer approach: simulation vs. real hardware

```
┌─────────────────────────────────────────────────────────────┐
│  ROS2 Application Layer                                     │
│  IK Node  →  /deltaflex_position_controller/commands       │
└──────────────────────┬──────────────────────────────────────┘
                       │ JointGroupPositionController
          ┌────────────┴────────────┐
          │                         │
   ┌──────▼──────┐          ┌───────▼───────────────┐
   │ Simulation  │          │ Custom HW Iface        │
   │ GazeboSystem│          │ (GRBL/serial)          │
   │ (Harmonic)  │          │                        │
   └─────────────┘          └────────────────────────┘
```

In simulation, `gz_ros2_control/GazeboSimSystem` handles the hardware interface automatically — Gazebo's physics engine provides the state feedback. For real hardware you swap the plugin for your custom implementation without changing anything above the hardware interface layer. This is the core value proposition of `ros2_control`.

### The open-chain URDF trick for parallel robots

URDF only models **kinematic trees** — directed acyclic graphs with a single parent per link. A true Delta robot has three closed kinematic chains (the forearm parallelogram loops). URDF cannot represent these loops natively.

The standard workaround used in production delta robot packages is:

1. **Model only the actuated tree.** Define three upper arms as `revolute` joints hanging off the base. Each upper arm is connected to a forearm link via a `revolute` or `fixed` joint.
2. **Terminate the forearms at phantom end-effector frames.** In simulation, the physics will not enforce the parallelogram constraint — the three forearms will dangle independently. This is geometrically wrong for dynamics but perfectly valid for:
   - Position control of the motor joints (which is all `ros2_control` needs)
   - Visualization/RVIZ rendering with joint state broadcaster
   - Collision geometry for the motors and upper arms
3. **Enforce the constraint in software.** Your IK node is the constraint. Given a Cartesian target, it computes the three motor angles that, if the physical linkage were closed, would position the end-effector at the target. The URDF is just a kinematic scaffolding for the control stack.

For dynamics simulation fidelity (e.g., studying vibration of the compliant joints), you would move to SDF with `<joint type="ball">` closed-loop support in Gazebo's physics engine. For position control development, the URDF open-chain approximation is standard practice and entirely sufficient.

---

> ### Confirmed Parameters from Paper (Parmiggiani et al.)
> *The following values are extracted directly from the DeltaFlex paper and supersede any estimates elsewhere in this guide.*
>
> | Parameter | Value | Notes |
> |-----------|-------|-------|
> | Transmission ratio | **3.8:1** | S2M timing belt + pulley — **not** direct drive |
> | Motor | NEMA 17, 1.8°/step | 200 steps/rev full step |
> | Holding torque | **0.59 Nm** | Motor datasheet (winding: 2.80 V) |
> | Effective joint torque | ~2.2 Nm | 0.59 Nm × 3.8 gear ratio |
> | Workspace (XY) | **±50 mm** | From repeatability test (Fig. 7, page 8/12) |
> | Workspace (Z) | **±60 mm** | From repeatability test |
> | Repeatability | **< 30 µm** | Average positioning error |
> | Stiffness (z-axis, perpendicular) | **0.41 N/mm** | From paper results (page 10) |
> | Stiffness (x-axis, tangential) | **0.19 N/mm** | From paper results (page 10) |
> | Electronics | Arduino UNO WiFi Rev.2 (ATmega4809) | *Not* standard Uno ATmega328P |
> | Stepper drivers | A4988 | On 3-Axis CNC/Stepper Motor Shield |
> | Limit switches | Omron D2MQ-4L-105-1 | One per axis, for homing |
> | Material | Duraform PA (Nylon PA12 / Polyamide 12) | SLS printed |
> | Flexure thickness | 0.7 mm | |
> | Young's modulus (effective) | **1000 MPa** | 27% below datasheet (1387 MPa) due to SLS |
> | Base rotational joint | Two-stage cross flex-pivot | |
> | Forearm spherical joints | Modified Tetra2 (Rommers et al. 2021) | Precision Engineering |

---

## 2. ROS2 Package Architecture

```
deltaflex_ros2/
├── deltaflex_description/        # URDF/Xacro, meshes, materials
│   ├── urdf/
│   │   ├── deltaflex.urdf.xacro
│   │   └── deltaflex.ros2_control.xacro
│   ├── meshes/
│   │   ├── base_link.stl
│   │   ├── upper_arm.stl
│   │   └── forearm.stl
│   ├── launch/
│   │   └── view_robot.launch.py
│   └── CMakeLists.txt
│
├── deltaflex_bringup/            # Top-level launch files
│   ├── launch/
│   │   ├── sim.launch.py         # Gazebo Harmonic simulation
│   │   └── real.launch.py        # Real hardware bringup
│   ├── config/
│   │   └── controllers.yaml
│   └── CMakeLists.txt
│
├── deltaflex_kinematics/         # IK/FK as ROS2 services
│   ├── deltaflex_kinematics/
│   │   ├── __init__.py
│   │   └── delta_ik.py           # Pure Python IK/FK math
│   ├── srv/
│   │   ├── IKSolve.srv
│   │   └── FKSolve.srv
│   ├── scripts/
│   │   └── ik_service_node.py
│   └── CMakeLists.txt
│
├── deltaflex_controllers/        # Cartesian control node
│   ├── deltaflex_controllers/
│   │   └── cartesian_controller.py
│   ├── scripts/
│   │   └── cartesian_controller_node.py
│   └── CMakeLists.txt
│
└── deltaflex_hardware/           # Real hardware interface
    ├── include/deltaflex_hardware/
    │   └── deltaflex_hardware_interface.hpp
    ├── src/
    │   └── deltaflex_hardware_interface.cpp
    ├── firmware/
    │   └── deltaflex_microros/
    │       └── deltaflex_microros.ino
    ├── deltaflex_hardware.xml    # pluginlib export
    └── CMakeLists.txt
```

Each package is a standard `ament_cmake` or `ament_python` package. All packages live inside a single `deltaflex_ros2` metapackage directory but can be built independently.

---

## 3. URDF / Xacro Model

### File: `deltaflex_description/urdf/deltaflex.urdf.xacro`

The URDF uses only standard URDF primitives — no mesh files required to get started. The robot is modelled as a simple open kinematic chain: three actuated revolute joints (the motor shafts) plus three passive revolute elbow joints. The `<dynamics>` tag on every revolute joint accepts a `spring_stiffness` parameter (N·m/rad) — set `joint_spring_k` to `0.0` for a rigid model, or a small positive value (e.g. `0.05`) to approximate the restoring force of the compliant flexures in Gazebo. The parallelogram closure constraint is enforced by the IK node, not by the URDF.

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="deltaflex">

  <!-- ================================================================
       Geometry parameters — measure from STEP files
       ================================================================ -->
  <xacro:property name="base_r"      value="0.100"/>  <!-- base circumradius [m] -->
  <xacro:property name="ee_r"        value="0.040"/>  <!-- EE circumradius [m] -->
  <xacro:property name="upper_len"   value="0.120"/>  <!-- upper arm length [m] -->
  <xacro:property name="lower_len"   value="0.200"/>  <!-- forearm length [m] -->

  <!-- ================================================================
       Spring stiffness for all revolute joints [N·m/rad]
       Set to 0.0 for rigid (standard delta), >0 to approximate
       the compliant flexures (paper: ~0.05–0.10 Nm/rad estimated).
       ================================================================ -->
  <xacro:property name="joint_spring_k"   value="0.0"/>   <!-- Nm/rad -->
  <xacro:property name="joint_damping"    value="0.01"/>  <!-- Nm·s/rad -->
  <xacro:property name="joint_friction"   value="0.002"/> <!-- Nm -->

  <!-- ================================================================
       Inertia helper — solid cylinder
       ================================================================ -->
  <xacro:macro name="cyl_inertia" params="m r h">
    <inertial>
      <mass value="${m}"/>
      <inertia ixx="${m*(3*r*r+h*h)/12}" ixy="0" ixz="0"
               iyy="${m*(3*r*r+h*h)/12}" iyz="0"
               izz="${m*r*r/2}"/>
    </inertial>
  </xacro:macro>

  <!-- ================================================================
       WORLD + BASE
       ================================================================ -->
  <link name="world"/>
  <joint name="world_to_base" type="fixed">
    <parent link="world"/>
    <child  link="base_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>

  <link name="base_link">
    <visual>
      <geometry><cylinder radius="${base_r}" length="0.015"/></geometry>
      <material name="gray"><color rgba="0.55 0.55 0.55 1"/></material>
    </visual>
    <collision>
      <geometry><cylinder radius="${base_r}" length="0.015"/></geometry>
    </collision>
    <inertial>
      <mass value="0.50"/>
      <inertia ixx="0.0012" ixy="0" ixz="0"
               iyy="0.0012" iyz="0" izz="0.0025"/>
    </inertial>
  </link>

  <!-- ================================================================
       ARM MACRO — instantiated 3× at 90°, 210°, 330°
       Each arm: motor_link → joint_N (revolute, actuated) →
                 upper_arm  → elbow_N (revolute, passive) →
                 forearm    → wrist_N (revolute, passive) →
                 ee_attach point
       ================================================================ -->
  <xacro:macro name="delta_arm" params="id angle_deg">

    <!-- Motor body (fixed to base at the attachment point) -->
    <link name="motor_${id}">
      <visual>
        <geometry><box size="0.042 0.042 0.048"/></geometry>
        <material name="blue"><color rgba="0.2 0.3 0.8 1"/></material>
      </visual>
      <collision>
        <geometry><box size="0.042 0.042 0.048"/></geometry>
      </collision>
      <inertial>
        <mass value="0.28"/>
        <inertia ixx="0.0001" ixy="0" ixz="0"
                 iyy="0.0001" iyz="0" izz="0.0001"/>
      </inertial>
    </link>

    <joint name="base_to_motor_${id}" type="fixed">
      <parent link="base_link"/>
      <child  link="motor_${id}"/>
      <origin xyz="${base_r*cos(radians(angle_deg))}
                   ${base_r*sin(radians(angle_deg))}
                   -0.024"
              rpy="0 0 ${radians(angle_deg)}"/>
    </joint>

    <!-- ACTUATED joint — motor shaft -->
    <link name="upper_arm_${id}">
      <visual>
        <origin xyz="0 0 ${-upper_len/2}" rpy="0 0 0"/>
        <geometry><cylinder radius="0.007" length="${upper_len}"/></geometry>
        <material name="orange"><color rgba="0.9 0.5 0.1 1"/></material>
      </visual>
      <collision>
        <origin xyz="0 0 ${-upper_len/2}" rpy="0 0 0"/>
        <geometry><cylinder radius="0.007" length="${upper_len}"/></geometry>
      </collision>
      <xacro:cyl_inertia m="0.04" r="0.007" h="${upper_len}"/>
    </link>

    <joint name="joint_${id}" type="revolute">
      <parent link="motor_${id}"/>
      <child  link="upper_arm_${id}"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <limit lower="${-pi/2}" upper="${pi/2}" effort="10.0" velocity="3.14"/>
      <!-- spring_reference=0: neutral = horizontal arm -->
      <dynamics damping="${joint_damping}"
                friction="${joint_friction}"
                spring_stiffness="${joint_spring_k}"
                spring_reference="0.0"/>
    </joint>

    <!-- PASSIVE elbow joint (upper arm → forearm) -->
    <link name="forearm_${id}">
      <visual>
        <origin xyz="0 0 ${-lower_len/2}" rpy="0 0 0"/>
        <geometry><cylinder radius="0.005" length="${lower_len}"/></geometry>
        <material name="silver"><color rgba="0.75 0.75 0.75 1"/></material>
      </visual>
      <collision>
        <origin xyz="0 0 ${-lower_len/2}" rpy="0 0 0"/>
        <geometry><cylinder radius="0.005" length="${lower_len}"/></geometry>
      </collision>
      <xacro:cyl_inertia m="0.015" r="0.005" h="${lower_len}"/>
    </link>

    <!-- Elbow: in reality a spherical flexure — modelled as revolute for simplicity -->
    <joint name="elbow_${id}" type="revolute">
      <parent link="upper_arm_${id}"/>
      <child  link="forearm_${id}"/>
      <origin xyz="0 0 ${-upper_len}" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <limit lower="${-pi/2}" upper="${pi/2}" effort="5.0" velocity="3.14"/>
      <dynamics damping="${joint_damping}"
                friction="${joint_friction}"
                spring_stiffness="${joint_spring_k}"
                spring_reference="0.0"/>
    </joint>

    <!-- EE attachment frame at tip of each forearm -->
    <link name="ee_attach_${id}">
      <inertial>
        <mass value="0.001"/>
        <inertia ixx="1e-7" ixy="0" ixz="0"
                 iyy="1e-7" iyz="0" izz="1e-7"/>
      </inertial>
    </link>

    <joint name="forearm_${id}_to_ee_attach" type="fixed">
      <parent link="forearm_${id}"/>
      <child  link="ee_attach_${id}"/>
      <origin xyz="0 0 ${-lower_len}" rpy="0 0 0"/>
    </joint>

  </xacro:macro>

  <!-- Instantiate 3 arms -->
  <xacro:delta_arm id="1" angle_deg="90"/>
  <xacro:delta_arm id="2" angle_deg="210"/>
  <xacro:delta_arm id="3" angle_deg="330"/>

  <!-- ================================================================
       END-EFFECTOR PLATFORM
       Attached to ee_attach_1 (visualisation reference).
       Note: in closed-chain reality all 3 ee_attach frames
       coincide — enforced by IK, not by URDF.
       ================================================================ -->
  <link name="end_effector">
    <visual>
      <geometry><cylinder radius="${ee_r}" length="0.008"/></geometry>
      <material name="red"><color rgba="0.8 0.15 0.15 1"/></material>
    </visual>
    <collision>
      <geometry><cylinder radius="${ee_r}" length="0.008"/></geometry>
    </collision>
    <inertial>
      <mass value="0.020"/>
      <inertia ixx="1.5e-5" ixy="0" ixz="0"
               iyy="1.5e-5" iyz="0" izz="3.2e-5"/>
    </inertial>
  </link>

  <joint name="ee_attach_1_to_ee" type="fixed">
    <parent link="ee_attach_1"/>
    <child  link="end_effector"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>

  <!-- ================================================================
       ros2_control TAGS
       ================================================================ -->
  <xacro:include filename="$(find deltaflex_description)/urdf/deltaflex.ros2_control.xacro"/>

</robot>
```

### File: `deltaflex_description/urdf/deltaflex.ros2_control.xacro`

This file is conditionally included so you can swap between simulation and real hardware by passing a `use_sim` argument.

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

  <xacro:arg name="use_sim" default="true"/>

  <!-- ============================================================
       SIMULATION hardware interface (Gazebo Harmonic)
       ============================================================ -->
  <xacro:if value="$(arg use_sim)">
    <ros2_control name="DeltaFlexSystem" type="system">
      <hardware>
        <plugin>gz_ros2_control/GazeboSimSystem</plugin>
      </hardware>

      <joint name="joint_1">
        <command_interface name="position">
          <param name="min">-1.5708</param>
          <param name="max">1.5708</param>
          <param name="initial_value">0.0</param>
        </command_interface>
        <state_interface name="position"/>
        <state_interface name="velocity"/>
      </joint>

      <joint name="joint_2">
        <command_interface name="position">
          <param name="min">-1.5708</param>
          <param name="max">1.5708</param>
          <param name="initial_value">0.0</param>
        </command_interface>
        <state_interface name="position"/>
        <state_interface name="velocity"/>
      </joint>

      <joint name="joint_3">
        <command_interface name="position">
          <param name="min">-1.5708</param>
          <param name="max">1.5708</param>
          <param name="initial_value">0.0</param>
        </command_interface>
        <state_interface name="position"/>
        <state_interface name="velocity"/>
      </joint>
    </ros2_control>

    <!-- Gazebo plugin — points to the controller YAML -->
    <gazebo>
      <plugin filename="libgz_ros2_control-system.so"
              name="gz_ros2_control::GazeboSimROS2ControlPlugin">
        <parameters>$(find deltaflex_bringup)/config/controllers.yaml</parameters>
        <!-- Tune position_proportional_gain to set joint response speed.
             Default 0.1 gives a time constant of ~0.1 s at 100 Hz. -->
        <position_proportional_gain>0.5</position_proportional_gain>
      </plugin>
    </gazebo>
  </xacro:if>

  <!-- ============================================================
       REAL HARDWARE interface — custom plugin
       ============================================================ -->
  <xacro:unless value="$(arg use_sim)">
    <ros2_control name="DeltaFlexSystem" type="system">
      <hardware>
        <plugin>deltaflex_hardware/DeltaFlexHardwareInterface</plugin>
        <param name="serial_port">/dev/ttyUSB0</param>
        <param name="baud_rate">115200</param>
        <param name="steps_per_rev">1600</param>  <!-- 200 * 8 microsteps (1/8 mode, RECOMMENDED); gear_ratio=3.8 applied via $100 in GRBL -->
      </hardware>

      <joint name="joint_1">
        <command_interface name="position"/>
        <state_interface name="position"/>
      </joint>
      <joint name="joint_2">
        <command_interface name="position"/>
        <state_interface name="position"/>
      </joint>
      <joint name="joint_3">
        <command_interface name="position"/>
        <state_interface name="position"/>
      </joint>
    </ros2_control>
  </xacro:unless>

</robot>
```

### Notes on inertia and collisions

Gazebo Harmonic requires valid inertia tensors on every link that participates in dynamics. The macros above provide approximate values — use a CAD tool (FreeCAD, Meshlab, or `xacro` inertia calculators) to compute accurate values from the actual STL mesh once you export it from the STEP files in the DeltaFlex repository.

---

## 4. Delta Robot Kinematics (IK/FK)

### Mathematical formulation

The DeltaFlex follows standard Delta robot geometry. Define the following parameters (all in metres, measure from the STEP files):

| Symbol | Description | Estimated value |
|--------|-------------|-----------------|
| `f`    | Base circumradius (centre to vertex) | 0.10 m |
| `e`    | End-effector circumradius | 0.04 m |
| `rf`   | Upper arm length | 0.12 m |
| `re`   | Forearm length | 0.20 m |

The three motor axes are co-planar in the base, spaced 120° apart. For arm *i*, the IK reduces to a 2D problem in the arm's vertical plane after rotating the end-effector target point by the arm's angle offset (0°, 120°, 240°).

**For arm 1 (0° / in the XZ plane of the arm's local frame):**

Given end-effector position \((x_0, y_0, z_0)\) in world frame (Z pointing down is the convention for delta robots; Z = 0 at the base):

\[
y_1 = -\frac{f}{2\sqrt{3}}, \quad y_0' = y_0 - \frac{e}{2\sqrt{3}}
\]

\[
a = \frac{x_0^2 + y_0'^2 + z_0^2 + r_f^2 - r_e^2 - y_1^2}{2 z_0}
\]

\[
b = \frac{y_1 - y_0'}{z_0}
\]

\[
d = -(a + b y_1)^2 + r_f^2(b^2 + 1)
\]

If \(d < 0\): the target is outside the workspace.

\[
y_j = \frac{y_1 - ab - \sqrt{d}}{b^2 + 1}, \quad z_j = a + b y_j
\]

\[
\theta_1 = \arctan2(-z_j,\ y_j - y_1)
\]

Arms 2 and 3 use the same formula after rotating \((x_0, y_0)\) by −120° and −240° respectively into arm 1's frame.

### File: `deltaflex_kinematics/deltaflex_kinematics/delta_ik.py`

```python
"""
Delta robot inverse kinematics for DeltaFlex.

Coordinate convention:
  - Z is positive downward (robot hangs from ceiling mount)
  - Home position: EE at (0, 0, z_home) where z_home is determined by geometry
  - All angles in radians; positive angle = arm rotates downward

Measure f, e, rf, re from the STEP files in the DeltaFlex repository:
  https://github.com/made-iit/deltaflex
"""

import numpy as np
from typing import Tuple

# ──────────────────────────────────────────────────
# Robot parameters — MUST be measured from hardware
# ──────────────────────────────────────────────────
F  = 0.10   # base circumradius [m]   (centre to vertex of base triangle)
E  = 0.04   # EE circumradius [m]     (centre to vertex of EE triangle)
RF = 0.12   # upper arm length [m]
RE = 0.20   # forearm length [m]

# Derived constants
_SQRT3 = np.sqrt(3.0)
_F_OVER_2SQRT3 = F / (2.0 * _SQRT3)   # = F * sqrt(3) / 6
_E_OVER_2SQRT3 = E / (2.0 * _SQRT3)


def _ik_single_arm(x0: float, y0: float, z0: float) -> float:
    """
    IK for a single arm aligned in the Y-Z plane (arm 1 canonical frame).

    The world-frame end-effector position must be pre-rotated into the
    canonical arm frame before calling this function.

    Parameters
    ----------
    x0, y0, z0 : float
        End-effector position in the canonical arm frame (metres).
        z0 must be negative (below base) for a physically reachable pose.

    Returns
    -------
    theta : float
        Motor joint angle in radians.

    Raises
    ------
    ValueError
        If the target is outside the reachable workspace.
    """
    y1 = -_F_OVER_2SQRT3        # pivot point offset on base
    y0 = y0 - _E_OVER_2SQRT3   # shift by EE triangle offset

    # Coefficients of the circle-intersection equation
    a = (x0**2 + y0**2 + z0**2 + RF**2 - RE**2 - y1**2) / (2.0 * z0)
    b = (y1 - y0) / z0

    discriminant = -(a + b * y1)**2 + RF**2 * (b**2 + 1.0)

    if discriminant < 0:
        raise ValueError(
            f"IK: no solution — target ({x0:.4f}, {y0:.4f}, {z0:.4f}) "
            f"is outside workspace (discriminant={discriminant:.6f})"
        )

    y_j = (y1 - a * b - np.sqrt(discriminant)) / (b**2 + 1.0)
    z_j = a + b * y_j

    return np.arctan2(-z_j, y_j - y1)


def _rotate_xy(x: float, y: float, angle_deg: float) -> Tuple[float, float]:
    """Rotate a point (x, y) in the horizontal plane by angle_deg degrees."""
    angle_rad = np.radians(angle_deg)
    cos_a, sin_a = np.cos(angle_rad), np.sin(angle_rad)
    return x * cos_a - y * sin_a, x * sin_a + y * cos_a


def inverse_kinematics(x: float, y: float, z: float) -> np.ndarray:
    """
    Compute all three motor joint angles for a given end-effector position.

    Parameters
    ----------
    x, y, z : float
        Desired end-effector position in the robot base frame (metres).
        z is positive-downward; typical working range is z ∈ [0.15, 0.30].

    Returns
    -------
    np.ndarray, shape (3,)
        Motor angles [theta_1, theta_2, theta_3] in radians.
        theta = 0 corresponds to the upper arm horizontal.

    Raises
    ------
    ValueError
        If the target is unreachable for any arm.
    """
    # Arm 1: no rotation needed (canonical frame aligned with Y axis)
    theta_1 = _ik_single_arm(x, y, z)

    # Arm 2: rotate EE target by +120° into arm 2's canonical frame
    x2, y2 = _rotate_xy(x, y, 120.0)
    theta_2 = _ik_single_arm(x2, y2, z)

    # Arm 3: rotate EE target by +240° (= -120°) into arm 3's canonical frame
    x3, y3 = _rotate_xy(x, y, 240.0)
    theta_3 = _ik_single_arm(x3, y3, z)

    return np.array([theta_1, theta_2, theta_3])


def forward_kinematics(theta: np.ndarray,
                       tol: float = 1e-6,
                       max_iter: int = 100) -> np.ndarray:
    """
    Numerical forward kinematics via Newton-Raphson.

    Finds the end-effector position (x, y, z) such that
    inverse_kinematics(x, y, z) ≈ theta.

    Parameters
    ----------
    theta : np.ndarray, shape (3,)
        Motor joint angles in radians.
    tol : float
        Convergence tolerance (metres).
    max_iter : int
        Maximum Newton-Raphson iterations.

    Returns
    -------
    np.ndarray, shape (3,)
        End-effector position [x, y, z] in metres.
    """
    # Initial guess: home position
    pos = np.array([0.0, 0.0, -(RF + RE) * 0.85])

    for _ in range(max_iter):
        try:
            theta_current = inverse_kinematics(*pos)
        except ValueError:
            # If outside workspace, step back toward home
            pos *= 0.95
            continue

        error = theta_current - theta
        if np.linalg.norm(error) < tol:
            return pos

        # Numerical Jacobian (finite differences)
        eps = 1e-5
        J = np.zeros((3, 3))
        for j in range(3):
            dp = pos.copy()
            dp[j] += eps
            try:
                J[:, j] = (inverse_kinematics(*dp) - theta_current) / eps
            except ValueError:
                J[:, j] = 0.0

        try:
            delta = np.linalg.solve(J, -error)
        except np.linalg.LinAlgError:
            break

        pos += delta

    raise RuntimeError(f"FK: Newton-Raphson did not converge after {max_iter} iterations")


# Workspace limits confirmed from paper (Parmiggiani et al.), repeatability test Fig. 7
WORKSPACE_X_MAX = 0.050  # m (from paper: ±50 mm)
WORKSPACE_Y_MAX = 0.050  # m
WORKSPACE_Z_MAX = 0.060  # m (from paper: ±60 mm)
WORKSPACE_Z_MIN = -0.060 # m


def workspace_check(x: float, y: float, z: float,
                    r_max: float = WORKSPACE_X_MAX,
                    z_min: float = WORKSPACE_Z_MIN,
                    z_max: float = WORKSPACE_Z_MAX) -> bool:
    """
    Fast workspace pre-check before calling IK.

    Default limits are from the DeltaFlex paper (Parmiggiani et al.), based on
    the 6 reference configurations tested in the repeatability experiment (Fig. 7):
      XY: ±50 mm radial, Z: ±60 mm.
    Note: the paper uses Z-positive-downward convention; adjust sign convention
    to match your IK frame if needed.
    """
    r = np.sqrt(x**2 + y**2)
    return r <= r_max and z_min <= z <= z_max
```

### ROS2 Service definitions

**File: `deltaflex_kinematics/srv/IKSolve.srv`**
```
# Request
float64 x
float64 y
float64 z
---
# Response
float64[] joint_angles   # [theta_1, theta_2, theta_3] in radians
bool success
string message
```

**File: `deltaflex_kinematics/srv/FKSolve.srv`**
```
# Request
float64[] joint_angles   # [theta_1, theta_2, theta_3] in radians
---
# Response
float64 x
float64 y
float64 z
bool success
string message
```

### File: `deltaflex_kinematics/scripts/ik_service_node.py`

```python
#!/usr/bin/env python3
"""
IK/FK service node for DeltaFlex.

Services:
  /deltaflex/ik  (deltaflex_kinematics/IKSolve)
  /deltaflex/fk  (deltaflex_kinematics/FKSolve)
"""

import rclpy
from rclpy.node import Node
import numpy as np

from deltaflex_kinematics.srv import IKSolve, FKSolve
from deltaflex_kinematics.delta_ik import (
    inverse_kinematics, forward_kinematics, workspace_check
)


class IKServiceNode(Node):
    def __init__(self):
        super().__init__('deltaflex_ik_service')

        self._ik_srv = self.create_service(
            IKSolve, '/deltaflex/ik', self._handle_ik)
        self._fk_srv = self.create_service(
            FKSolve, '/deltaflex/fk', self._handle_fk)

        self.get_logger().info('IK/FK services ready.')

    def _handle_ik(self, request, response):
        x, y, z = request.x, request.y, request.z

        if not workspace_check(x, y, z):
            response.success = False
            response.message = (
                f'Position ({x:.4f}, {y:.4f}, {z:.4f}) '
                f'failed workspace pre-check.')
            response.joint_angles = [0.0, 0.0, 0.0]
            self.get_logger().warn(response.message)
            return response

        try:
            angles = inverse_kinematics(x, y, z)
            response.joint_angles = angles.tolist()
            response.success = True
            response.message = 'OK'
        except ValueError as e:
            response.success = False
            response.message = str(e)
            response.joint_angles = [0.0, 0.0, 0.0]
            self.get_logger().warn(f'IK failed: {e}')

        return response

    def _handle_fk(self, request, response):
        theta = np.array(request.joint_angles)

        try:
            pos = forward_kinematics(theta)
            response.x, response.y, response.z = pos
            response.success = True
            response.message = 'OK'
        except (ValueError, RuntimeError) as e:
            response.success = False
            response.message = str(e)
            response.x = response.y = response.z = 0.0
            self.get_logger().warn(f'FK failed: {e}')

        return response


def main(args=None):
    rclpy.init(args=args)
    node = IKServiceNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## 5. ros2_control Configuration

### File: `deltaflex_bringup/config/controllers.yaml`

```yaml
# ──────────────────────────────────────────────────
# Controller Manager
# ──────────────────────────────────────────────────
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz — match your stepper interrupt frequency

    # Declare all controllers that can be loaded
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    deltaflex_position_controller:
      type: forward_command_controller/ForwardCommandController

# ──────────────────────────────────────────────────
# Joint State Broadcaster
# Publishes: /joint_states (sensor_msgs/JointState)
# Required by robot_state_publisher and RVIZ.
# ──────────────────────────────────────────────────
joint_state_broadcaster:
  ros__parameters:
    joints:
      - joint_1
      - joint_2
      - joint_3

# ──────────────────────────────────────────────────
# Forward Command Controller
#
# Type: forward_command_controller/ForwardCommandController
# Why: Directly forwards Float64MultiArray commands to the
#      position command interfaces of the three joints.
#      This is appropriate here because the IK node (not
#      the controller) handles all Cartesian-to-joint mapping.
#      JointGroupPositionController is an alias; use
#      ForwardCommandController for maximum clarity.
#
# Subscribes: /deltaflex_position_controller/commands
#             (std_msgs/Float64MultiArray, data=[θ1, θ2, θ3])
# ──────────────────────────────────────────────────
deltaflex_position_controller:
  ros__parameters:
    joints:
      - joint_1
      - joint_2
      - joint_3
    interface_name: position
```

### Why `ForwardCommandController` instead of `JointTrajectoryController`

`JointTrajectoryController` adds trajectory interpolation, velocity/acceleration limits, and time parameterisation — all useful features, but they require you to send `JointTrajectory` messages and define joint velocity/acceleration limits in the URDF. For a delta robot where you are managing trajectory interpolation yourself in the Cartesian controller node, `ForwardCommandController` is simpler and lower-latency. You can always switch to `JointTrajectoryController` later once the system is running.

### Activating controllers at launch

```bash
# After launching the bringup:
ros2 control list_controllers

# Expected output:
# joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
# deltaflex_position_controller[forward_command_controller/ForwardCommandController] active

# Manual activation (if not auto-activated in launch file):
ros2 control set_controller_state deltaflex_position_controller active
```

---

## 6. Gazebo Simulation Setup

### How `gz_ros2_control` works in Gazebo Harmonic

When Gazebo loads the robot model it finds the `libgz_ros2_control-system.so` plugin declared in the URDF `<gazebo>` tag. This plugin:

1. Reads the `<ros2_control>` tags in the robot description.
2. Instantiates the `GazeboSimSystem` hardware interface, which wires each declared joint's command/state interfaces to Gazebo's joint handle API.
3. Starts a `controller_manager` node, loading the controllers specified in the YAML file.
4. Runs the `ros2_control` control loop at `update_rate` Hz, calling `read()` → `update()` → `write()` each tick.

The `position` command interface in Gazebo Harmonic is implemented as a **P controller** inside Gazebo that commands joint velocity to track the position setpoint. The proportional gain defaults to `0.1`; set `position_proportional_gain` to `0.5`–`1.0` for a snappier response without oscillation.

### Launch file: `deltaflex_bringup/launch/sim.launch.py`

```python
"""
sim.launch.py

Starts:
  1. Gazebo Harmonic (gz sim) with an empty world
  2. Robot description publisher (robot_state_publisher)
  3. Spawn the DeltaFlex model into Gazebo
  4. ros2_control controller manager (started by gz_ros2_control plugin)
  5. joint_state_broadcaster
  6. deltaflex_position_controller
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ── Paths ────────────────────────────────────────────────────────────────
    description_pkg = get_package_share_directory('deltaflex_description')
    bringup_pkg     = get_package_share_directory('deltaflex_bringup')

    xacro_file = os.path.join(description_pkg, 'urdf', 'deltaflex.urdf.xacro')
    world_file = os.path.join(bringup_pkg, 'worlds', 'empty.sdf')

    # ── Arguments ────────────────────────────────────────────────────────────
    declare_gui = DeclareLaunchArgument(
        'gui', default_value='true',
        description='Start Gazebo GUI (gz-gui)'
    )
    declare_verbose = DeclareLaunchArgument(
        'verbose', default_value='false',
        description='Verbose Gazebo output'
    )

    gui     = LaunchConfiguration('gui')
    verbose = LaunchConfiguration('verbose')

    # ── Robot description via xacro ──────────────────────────────────────────
    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ',
        xacro_file,
        ' use_sim:=true'
    ])
    robot_description = {'robot_description': robot_description_content}

    # ── robot_state_publisher ────────────────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    # ── Gazebo Harmonic ───────────────────────────────────────────────────────
    # Use 'gz sim' (not 'gazebo'). The -r flag starts the simulation immediately.
    gz_sim = ExecuteProcess(
        cmd=[
            'gz', 'sim', '-r', world_file,
            # Conditionally suppress GUI:
            # '--headless-rendering' can be added for CI
        ],
        output='screen',
    )

    # ── Spawn robot ───────────────────────────────────────────────────────────
    # gz_ros2_control_demos uses ros_gz_sim for spawning — same approach here.
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', '/robot_description',
            '-name',  'deltaflex',
            '-z',     '0.5',           # spawn 0.5 m above ground
        ],
        output='screen',
    )

    # ── Bridge: Gazebo clock → ROS2 /clock ───────────────────────────────────
    gz_clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    # ── Activate controllers after spawn ─────────────────────────────────────
    # Use RegisterEventHandler to wait for spawn to complete before activating.
    activate_jsb = ExecuteProcess(
        cmd=['ros2', 'control', 'set_controller_state',
             'joint_state_broadcaster', 'active'],
        output='screen',
    )

    activate_pos_ctrl = ExecuteProcess(
        cmd=['ros2', 'control', 'set_controller_state',
             'deltaflex_position_controller', 'active'],
        output='screen',
    )

    # Delay controller activation to allow controller_manager to start
    activate_controllers = TimerAction(
        period=3.0,
        actions=[activate_jsb, activate_pos_ctrl]
    )

    # ── IK service node ───────────────────────────────────────────────────────
    ik_node = Node(
        package='deltaflex_kinematics',
        executable='ik_service_node.py',
        name='deltaflex_ik_service',
        output='screen',
    )

    return LaunchDescription([
        declare_gui,
        declare_verbose,
        gz_sim,
        robot_state_publisher,
        gz_clock_bridge,
        spawn_robot,
        activate_controllers,
        ik_node,
    ])
```

### `worlds/empty.sdf`

```xml
<?xml version="1.0"?>
<sdf version="1.9">
  <world name="deltaflex_world">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="gz-sim-physics-system"
            name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system"
            name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system"
            name="gz::sim::systems::SceneBroadcaster"/>

    <!-- Ambient light -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <!-- Ground plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

  </world>
</sdf>
```

---

## 7. Real Hardware — Stepper Control

### Hardware overview

The DeltaFlex real-hardware stack uses an **Arduino UNO WiFi Rev.2** (ATmega4809) with a **3-Axis CNC/Stepper Motor Shield** carrying three **A4988** stepper drivers. The board runs **GRBL 1.1** firmware, which provides a proven, well-tested G-code motion controller for 3-axis stepper systems. The host PC (Ubuntu 24.04, ROS2 Jazzy) communicates with GRBL over USB serial at 115200 baud.

> **Important:** The paper (Parmiggiani et al.) specifies the **Arduino UNO WiFi Rev.2** (ATmega4809), which is *not* the standard Arduino Uno (ATmega328P). The ATmega4809 has 6 KB SRAM and 48 KB flash vs. the ATmega328P's 2 KB / 32 KB. GRBL 1.1 is compiled for the ATmega328P; on the ATmega4809 you must use a compatible GRBL port (e.g., [grbl-Mega-5X](https://github.com/fra589/grbl-Mega-5X) or a purpose-built ATmega4809 port). Adjust the `avrdude` MCU target accordingly (`-p atmega4809`).

**Why not micro-ROS on the board?**  
The micro-ROS client library requires approximately 30 KB of RAM minimum — even on the ATmega4809 (6 KB SRAM) this is infeasible. The correct solution is GRBL on the board (fits comfortably) combined with a custom `ros2_control` hardware interface on the host PC that translates joint commands into G-code over serial. This architecture keeps the firmware simple and battle-tested, while the full ROS2 stack runs on the Linux host where RAM is not a constraint.

**Signal path:**
```
ROS2 Jazzy (Linux PC)
  └─► ros2_control hardware interface (C++, host)
        └─► USB serial /dev/ttyACM0
              └─► custom firmware (Arduino Uno R4 WiFi, RA4M1)
                    └─► A4988 × 3 (3-Axis CNC/Stepper Motor Shield)
                          └─► S2M timing belt + pulley (3.8:1 ratio)
                                └─► NEMA 17 × 3
```

---

### 7.1 Step/angle conversion

For the DeltaFlex's NEMA 17 steppers:

- **Full step resolution:** 200 steps/revolution (1.8°/step)
- **Microstepping via A4988 MS1/MS2/MS3 jumpers on CNC Shield v3:**

| MS1 | MS2 | MS3 | Microstepping | Steps/rev |
|-----|-----|-----|---------------|-----------|
| LOW | LOW | LOW | Full step | 200 |
| HIGH | LOW | LOW | Half step | 400 |
| LOW | HIGH | LOW | 1/4 step | 800 |
| HIGH | HIGH | LOW | 1/8 step | 1600 |
| HIGH | HIGH | HIGH | 1/16 step | 3200 |

> **Note:** The microstepping jumpers on the CNC Shield v3 are located **underneath each A4988 driver socket**. You must remove the driver to change the jumper positions. All three jumpers shorted = 1/16 microstepping (3200 steps/rev); two jumpers shorted (MS1+MS2) = 1/8 microstepping (1600 steps/rev).

**Recommended: 1/8 microstepping (1600 steps/rev)** for use with the board. At 1/16 microstepping (3200 steps/rev) and moderate joint velocities (~1 rad/s), GRBL must generate ~1935 pulses/sec per axis (after applying the 3.8:1 gear ratio). At 1/8 microstepping this drops to ~968 pulses/sec per axis, giving GRBL comfortable timing margin and reducing the risk of missed steps during simultaneous 3-axis moves.

> **Critical — 3.8:1 transmission ratio:** The DeltaFlex uses an S2M timing belt and pulley with a **3.8:1 mechanical transmission ratio** between the motor shaft and the arm joint. The motor turns 3.8 revolutions for every 1 radian of joint travel. All step/angle calculations must include this factor. The guide originally assumed direct drive (1:1) — this was incorrect.

The corrected step/angle relationship:

\[
\text{steps} = \frac{\theta_{\text{rad}}}{2\pi} \times \text{steps\_per\_rev} \times \underbrace{3.8}_{\text{gear ratio}}
\]

For a joint command of 0.5 rad with 1600 steps/rev (1/8 microstepping) and 3.8:1 gear ratio:

\[
\text{steps} = \frac{0.5}{2\pi} \times 1600 \times 3.8 \approx 484 \text{ steps}
\]

In Python (angle-to-steps conversion):

```python
import math

MICROSTEP_FACTOR = 8          # 1/8 microstepping (MS1+MS2 shorted)
STEPS_PER_REV = 200 * MICROSTEP_FACTOR   # = 1600
GEAR_RATIO = 3.8              # S2M belt + pulley transmission

def angle_to_steps(angle_rad: float) -> int:
    """Convert joint angle [rad] to motor step count."""
    return round(angle_rad / (2 * math.pi) * STEPS_PER_REV * GEAR_RATIO)
```

Steps per radian at common microstepping settings:

| Microstepping | Steps/rev | Steps/rad (incl. 3.8:1) |
|---------------|-----------|-------------------------|
| Full step | 200 | 200 × 3.8 / (2π) = **120.96** |
| 1/8 step | 1600 | 1600 × 3.8 / (2π) = **967.7** |
| 1/16 step | 3200 | 3200 × 3.8 / (2π) = **1935.4** |

**CNC Shield v3 pin assignments (fixed by PCB):**
```
X axis: STEP=D2, DIR=D5    →  joint_1
Y axis: STEP=D3, DIR=D6    →  joint_2
Z axis: STEP=D4, DIR=D7    →  joint_3
ENABLE (all axes): D8       (active LOW — pull LOW to enable drivers)
```

---

### 7.2 Firmware Options for Arduino Uno R4 WiFi

The Arduino Uno R4 WiFi uses the **Renesas RA4M1** (ARM Cortex-M4, 256 KB flash, 32 KB RAM). This board **cannot run standard GRBL 1.1** (which targets AVR ATmega328P/4809). Two practical options exist:

#### Option A: Custom Serial-Command Firmware (Recommended)
Write a minimal Arduino sketch that receives simple serial commands from the Linux PC and drives the A4988 steppers via `AccelStepper`. This avoids the GRBL dependency entirely and gives full control.

```cpp
// deltaflex_firmware.ino — Arduino Uno R4 WiFi
// Receives: "M<j1_steps> <j2_steps> <j3_steps>\n"  (absolute step targets)
// Responds: "ok\n" after move completes (blocking) or "err\n"

#include <AccelStepper.h>

// CNC Shield v3 pin assignments
#define X_STEP 2  #define X_DIR 5
#define Y_STEP 3  #define Y_DIR 6
#define Z_STEP 4  #define Z_DIR 7
#define ENABLE_PIN 8

AccelStepper stepperX(AccelStepper::DRIVER, X_STEP, X_DIR);
AccelStepper stepperY(AccelStepper::DRIVER, Y_STEP, Y_DIR);
AccelStepper stepperZ(AccelStepper::DRIVER, Z_STEP, Z_DIR);

const float MAX_SPEED    = 800.0;  // steps/sec
const float ACCELERATION = 400.0;  // steps/sec²

void setup() {
  Serial.begin(115200);
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW);  // enable drivers (active LOW)

  stepperX.setMaxSpeed(MAX_SPEED);  stepperX.setAcceleration(ACCELERATION);
  stepperY.setMaxSpeed(MAX_SPEED);  stepperY.setAcceleration(ACCELERATION);
  stepperZ.setMaxSpeed(MAX_SPEED);  stepperZ.setAcceleration(ACCELERATION);
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.startsWith("M")) {
      long t1, t2, t3;
      if (sscanf(cmd.c_str(), "M%ld %ld %ld", &t1, &t2, &t3) == 3) {
        stepperX.moveTo(t1);
        stepperY.moveTo(t2);
        stepperZ.moveTo(t3);
        // Run until all steppers reach target
        while (stepperX.distanceToGo() != 0 ||
               stepperY.distanceToGo() != 0 ||
               stepperZ.distanceToGo() != 0) {
          stepperX.run();
          stepperY.run();
          stepperZ.run();
        }
        Serial.println("ok");
      } else {
        Serial.println("err");
      }
    } else if (cmd == "HOME") {
      stepperX.setCurrentPosition(0);
      stepperY.setCurrentPosition(0);
      stepperZ.setCurrentPosition(0);
      Serial.println("ok");
    } else if (cmd == "?") {
      Serial.print("X"); Serial.print(stepperX.currentPosition());
      Serial.print(" Y"); Serial.print(stepperY.currentPosition());
      Serial.print(" Z"); Serial.println(stepperZ.currentPosition());
    }
  }
}
```

Flash with arduino-cli:
```bash
# Install arduino-cli
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh
arduino-cli core install arduino:renesas_uno
arduino-cli lib install AccelStepper

# Compile and flash (adjust port)
arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi deltaflex_firmware/
arduino-cli upload  --fqbn arduino:renesas_uno:unor4wifi -p /dev/ttyACM0 deltaflex_firmware/
```

#### Option B: GRBL_ESP32 / grbl-LPC port (Advanced)
Some community GRBL ports exist for ARM Cortex-M targets. As of 2026, [GRBL-RA4M1](https://github.com/) ports are experimental. If you want G-code compatibility, consider using an **Arduino Mega 2560** (ATmega2560) with GRBL, which is fully supported. This keeps the ros2_control hardware interface G-code based as described in Section 7.4.

---

### 7.3 GRBL Configuration ($-settings)

> **Note:** If using the custom serial firmware (Option A, recommended for Uno R4 WiFi), skip this section entirely. The step count is computed in the ros2_control hardware interface and sent directly as `M<j1> <j2> <j3>`. The GRBL `$`-settings below apply only if you are using GRBL on an AVR-based board (e.g. Arduino Mega 2560).

GRBL thinks in units of "mm" internally, but we repurpose these units as **radians** for joint-space control. The key insight: if we set `$100` (steps per "mm") equal to `steps_per_rev / (2π)`, then sending `G1 X<angle_rad>` causes GRBL to move exactly `steps_per_rev/(2π) × angle_rad` steps — which is the correct step count for that joint angle.

Connect with a serial terminal after flashing:

```bash
screen /dev/ttyACM0 115200
# or
minicom -D /dev/ttyACM0 -b 115200
```

Then enter the following `$`-commands one at a time. GRBL responds with `ok` after each:

```
# ── DeltaFlex GRBL configuration ─────────────────────────────────────────────────────────────────────
#
# Unit mapping: 1 GRBL "mm" = 1 radian of joint travel
# steps_per_unit = steps_per_rev * gear_ratio / (2 * pi)
#
# Gear ratio (S2M belt + pulley): 3.8:1 (from paper, Parmiggiani et al.)
#
# For 1/8 microstepping: steps_per_rev = 200 * 8 = 1600
#   steps_per_unit = 1600 * 3.8 / (2 * 3.14159265) = 967.7 steps/rad
#   [RECOMMENDED]
#
# For 1/16 microstepping: steps_per_rev = 200 * 16 = 3200
#   steps_per_unit = 3200 * 3.8 / (2 * 3.14159265) = 1935.4 steps/rad

$100=967.7    # X (joint_1) steps per unit [steps/rad] — 1/8 microstepping, 3.8:1 ratio
$101=967.7    # Y (joint_2) steps per unit [steps/rad]
$102=967.7    # Z (joint_3) steps per unit [steps/rad]
# For 1/16 microstepping use: $100=$101=$102=1935.4

# Max rate [rad/min] — 500 rad/min ≈ 8.3 rad/s (conservative starting point)
$110=500       # X max rate
$111=500       # Y max rate
$112=500       # Z max rate

# Acceleration [rad/min²] — ramp up slowly to avoid missed steps
$120=50        # X acceleration
$121=50        # Y acceleration
$122=50        # Z acceleration

# Max travel [rad] — used only when soft limits are enabled
# 360 rad is a generous placeholder; tighten once homing is working
$130=360       # X max travel
$131=360       # Y max travel
$132=360       # Z max travel

# Safety settings — disable until homing is verified
$20=0          # Soft limits OFF (enable after homing: $20=1)
$21=0          # Hard limits OFF (enable when limit switches wired: $21=1)
$22=0          # Homing cycle OFF (enable when limit switches wired: $22=1)

# Step pulse width and step idle delay
$0=10          # Step pulse time [µs] — 10 µs is fine for A4988
$1=255         # Step idle delay [ms] — 255 = keep steppers energized always
```

To verify the settings were stored:

```
$$
```

GRBL will print all `$` values. Confirm `$100`–`$102` show `967.7` (1/8 microstepping, 3.8:1 gear ratio).

---

### 7.4 Custom ros2_control Hardware Interface (C++)

This is the primary ROS2 integration path. The hardware interface runs entirely on the Linux host PC; the Uno/GRBL firmware requires no modification beyond the `$`-settings above.

**File: `deltaflex_hardware/include/deltaflex_hardware/deltaflex_hardware_interface.hpp`**

```cpp
#pragma once

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

namespace deltaflex_hardware
{

class DeltaFlexHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DeltaFlexHardwareInterface)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Serial port handle
  int serial_fd_{-1};
  std::string serial_port_;
  int baud_rate_;
  int steps_per_rev_;
  double feed_rate_;   // rad/min sent as GRBL F-word

  // Joint state (virtual encoder — echoes commanded position)
  std::vector<double> hw_positions_;    // state interface
  std::vector<double> hw_commands_;     // command interface

  // Last commanded positions (to suppress redundant G-code)
  std::vector<double> last_commands_;

  // Helpers
  bool open_serial(const std::string & port, int baud);
  bool send_gcode(const std::string & line);
  std::string readline(int timeout_ms = 500);
  bool wait_for_ok(int timeout_ms = 1000);
};

}  // namespace deltaflex_hardware
```

**File: `deltaflex_hardware/src/deltaflex_hardware_interface.cpp`**

```cpp
#include "deltaflex_hardware/deltaflex_hardware_interface.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cmath>
#include <cstring>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <thread>

namespace deltaflex_hardware
{

// ── on_init ────────────────────────────────────────────────────────────────

hardware_interface::CallbackReturn
DeltaFlexHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Read parameters from URDF <param> tags
  serial_port_   = info_.hardware_parameters.at("serial_port");
  baud_rate_     = std::stoi(info_.hardware_parameters.at("baud_rate"));
  steps_per_rev_ = std::stoi(info_.hardware_parameters.at("steps_per_rev"));

  // feed_rate is optional — default 60 rad/min (= 1 rad/s)
  if (info_.hardware_parameters.count("feed_rate")) {
    feed_rate_ = std::stod(info_.hardware_parameters.at("feed_rate"));
  } else {
    feed_rate_ = 60.0;
  }

  hw_positions_.resize(info_.joints.size(), 0.0);
  hw_commands_.resize(info_.joints.size(), 0.0);
  last_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  RCLCPP_INFO(rclcpp::get_logger("DeltaFlexHW"),
              "Initialized: port=%s baud=%d steps_per_rev=%d feed_rate=%.1f",
              serial_port_.c_str(), baud_rate_, steps_per_rev_, feed_rate_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ── export interfaces ──────────────────────────────────────────────────────

std::vector<hardware_interface::StateInterface>
DeltaFlexHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (std::size_t i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_POSITION,
      &hw_positions_[i]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
DeltaFlexHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (std::size_t i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_POSITION,
      &hw_commands_[i]);
  }
  return command_interfaces;
}

// ── on_activate ───────────────────────────────────────────────────────────

hardware_interface::CallbackReturn
DeltaFlexHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
{
  if (!open_serial(serial_port_, baud_rate_)) {
    RCLCPP_ERROR(rclcpp::get_logger("DeltaFlexHW"),
                 "Cannot open serial port %s", serial_port_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // GRBL boots and sends a version string + idle status.
  // Wait 2 s for GRBL to initialise, then drain any buffered output.
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  // Drain startup messages
  {
    char buf[256];
    while (::read(serial_fd_, buf, sizeof(buf)) > 0) {}
  }

  // Unlock GRBL (clears alarm state after power-on or hard reset)
  if (!send_gcode("$X")) {
    RCLCPP_WARN(rclcpp::get_logger("DeltaFlexHW"), "Failed to send $X unlock");
  }
  wait_for_ok(500);

  // Set absolute positioning mode
  if (!send_gcode("G90")) {
    RCLCPP_ERROR(rclcpp::get_logger("DeltaFlexHW"), "Failed to send G90");
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!wait_for_ok(500)) {
    RCLCPP_ERROR(rclcpp::get_logger("DeltaFlexHW"), "No 'ok' from GRBL after G90");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Zero GRBL's work coordinate system at the current (home) position
  if (!send_gcode("G92 X0 Y0 Z0")) {
    RCLCPP_WARN(rclcpp::get_logger("DeltaFlexHW"), "Failed to send G92 zero");
  }
  wait_for_ok(500);

  RCLCPP_INFO(rclcpp::get_logger("DeltaFlexHW"),
              "Connected to GRBL on %s at %d baud. G90 absolute mode active.",
              serial_port_.c_str(), baud_rate_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ── on_deactivate ─────────────────────────────────────────────────────────

hardware_interface::CallbackReturn
DeltaFlexHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  // Send feed hold then soft-reset to stop any motion
  if (serial_fd_ >= 0) {
    send_gcode("!");   // GRBL feed hold (immediate)
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    // Soft reset: GRBL real-time command 0x18 (Ctrl-X)
    char reset_cmd = 0x18;
    ::write(serial_fd_, &reset_cmd, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    ::close(serial_fd_);
    serial_fd_ = -1;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ── read ──────────────────────────────────────────────────────────────────

hardware_interface::return_type
DeltaFlexHardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  // Virtual encoder: state = commanded position.
  // Open-loop steppers have no feedback — echo back the last command.
  // This is the correct pattern for ros2_control with open-loop actuators.
  for (std::size_t i = 0; i < hw_positions_.size(); i++) {
    hw_positions_[i] = hw_commands_[i];
  }
  return hardware_interface::return_type::OK;
}

// ── write ─────────────────────────────────────────────────────────────────

hardware_interface::return_type
DeltaFlexHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (serial_fd_ < 0) {
    RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("DeltaFlexHW"),
                          *rclcpp::Clock::make_shared(), 5000,
                          "Serial port not open");
    return hardware_interface::return_type::ERROR;
  }

  // Check if any joint command has changed (avoid flooding GRBL with
  // identical G-code lines at 100 Hz when the robot is stationary).
  bool changed = false;
  const double threshold = 1e-6;  // rad — smaller than one microstep
  for (std::size_t i = 0; i < hw_commands_.size(); i++) {
    if (std::isnan(last_commands_[i]) ||
        std::abs(hw_commands_[i] - last_commands_[i]) > threshold) {
      changed = true;
      break;
    }
  }
  if (!changed) {
    return hardware_interface::return_type::OK;
  }

  // Build a single G-code line for all three joints.
  //
  // GRBL coordinate mapping:
  //   X = joint_1 (CNC Shield X slot, STEP=D2 DIR=D5)
  //   Y = joint_2 (CNC Shield Y slot, STEP=D3 DIR=D6)
  //   Z = joint_3 (CNC Shield Z slot, STEP=D4 DIR=D7)
  //
  // Because we configured $100=$101=$102 = steps_per_rev * gear_ratio / (2π),
  // GRBL will convert position [rad] → steps correctly:
  //   steps = position_rad × (steps_per_rev * gear_ratio / (2π))
  // For 1/8 microstepping with 3.8:1 gear ratio: $100 = 967.7 steps/rad
  //
  // G90 = absolute positioning (set on activate, but repeat for safety)
  // G1  = linear interpolated move
  // F   = feed rate in GRBL units/min (= rad/min here)

  std::ostringstream gcode;
  gcode << std::fixed << std::setprecision(6);
  gcode << "G90 G1"
        << " X" << hw_commands_[0]
        << " Y" << hw_commands_[1]
        << " Z" << hw_commands_[2]
        << " F" << feed_rate_;

  if (!send_gcode(gcode.str())) {
    RCLCPP_WARN(rclcpp::get_logger("DeltaFlexHW"),
                "Failed to write G-code: %s", gcode.str().c_str());
    return hardware_interface::return_type::ERROR;
  }

  // Wait for GRBL 'ok' acknowledgement before returning.
  // This creates back-pressure: write() will block until GRBL has accepted
  // (queued) the command. GRBL has a 15-line planner buffer, so 'ok' is
  // returned as soon as the line is accepted into the buffer, not when
  // motion completes — latency is typically <1 ms at 115200 baud.
  if (!wait_for_ok(200)) {
    RCLCPP_WARN(rclcpp::get_logger("DeltaFlexHW"),
                "Timeout waiting for GRBL 'ok' after: %s", gcode.str().c_str());
    // Non-fatal: GRBL may still execute the move. Log and continue.
  }

  // Update last commands
  for (std::size_t i = 0; i < hw_commands_.size(); i++) {
    last_commands_[i] = hw_commands_[i];
  }

  return hardware_interface::return_type::OK;
}

// ── Private helpers ────────────────────────────────────────────────────────

bool DeltaFlexHardwareInterface::open_serial(const std::string & port, int baud)
{
  serial_fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (serial_fd_ < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("DeltaFlexHW"),
                 "open(%s) failed: %s", port.c_str(), strerror(errno));
    return false;
  }

  struct termios options;
  ::tcgetattr(serial_fd_, &options);

  // Map integer baud rate to termios speed constant
  speed_t speed = B115200;
  switch (baud) {
    case 9600:   speed = B9600;   break;
    case 19200:  speed = B19200;  break;
    case 38400:  speed = B38400;  break;
    case 57600:  speed = B57600;  break;
    case 115200: speed = B115200; break;
    default:
      RCLCPP_WARN(rclcpp::get_logger("DeltaFlexHW"),
                  "Unsupported baud rate %d — defaulting to 115200", baud);
      speed = B115200;
  }

  ::cfsetispeed(&options, speed);
  ::cfsetospeed(&options, speed);

  // 8N1, no flow control, raw mode
  options.c_cflag |=  (CLOCAL | CREAD);
  options.c_cflag &= ~PARENB;
  options.c_cflag &= ~CSTOPB;
  options.c_cflag &= ~CSIZE;
  options.c_cflag |=  CS8;
  options.c_cflag &= ~CRTSCTS;

  options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  options.c_iflag &= ~(IXON | IXOFF | IXANY);
  options.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
  options.c_oflag &= ~OPOST;

  options.c_cc[VMIN]  = 0;
  options.c_cc[VTIME] = 1;   // 0.1 s read timeout

  ::tcsetattr(serial_fd_, TCSANOW, &options);
  ::tcflush(serial_fd_, TCIOFLUSH);

  return true;
}

std::string DeltaFlexHardwareInterface::readline(int timeout_ms)
{
  std::string line;
  char c;
  auto deadline = std::chrono::steady_clock::now() +
                  std::chrono::milliseconds(timeout_ms);

  while (std::chrono::steady_clock::now() < deadline) {
    ssize_t n = ::read(serial_fd_, &c, 1);
    if (n == 1) {
      if (c == '\n') {
        // Strip trailing '\r' if present
        if (!line.empty() && line.back() == '\r') {
          line.pop_back();
        }
        return line;
      }
      line += c;
    } else {
      // No data yet — brief sleep to avoid busy-spin
      std::this_thread::sleep_for(std::chrono::microseconds(500));
    }
  }
  return line;  // timeout — return whatever was accumulated
}

bool DeltaFlexHardwareInterface::send_gcode(const std::string & line)
{
  if (serial_fd_ < 0) return false;
  std::string cmd = line + "\n";
  ssize_t written = ::write(serial_fd_, cmd.c_str(), cmd.size());
  return written == static_cast<ssize_t>(cmd.size());
}

bool DeltaFlexHardwareInterface::wait_for_ok(int timeout_ms)
{
  auto deadline = std::chrono::steady_clock::now() +
                  std::chrono::milliseconds(timeout_ms);

  while (std::chrono::steady_clock::now() < deadline) {
    std::string response = readline(50);
    if (response == "ok") {
      return true;
    }
    // GRBL may also send error messages or status reports
    if (response.rfind("error:", 0) == 0) {
      RCLCPP_WARN(rclcpp::get_logger("DeltaFlexHW"),
                  "GRBL error response: %s", response.c_str());
      return false;
    }
    // Ignore other lines (status reports starting with '<', etc.)
  }
  return false;  // timeout
}

}  // namespace deltaflex_hardware

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  deltaflex_hardware::DeltaFlexHardwareInterface,
  hardware_interface::SystemInterface)
```

**File: `deltaflex_hardware/deltaflex_hardware.xml`**

```xml
<library path="deltaflex_hardware">
  <class
    name="deltaflex_hardware/DeltaFlexHardwareInterface"
    type="deltaflex_hardware::DeltaFlexHardwareInterface"
    base_class_type="hardware_interface::SystemInterface">
    <description>
      Custom ros2_control hardware interface for DeltaFlex stepper motors.
      Communicates with firmware on Arduino Uno R4 WiFi via USB serial.
      Open-loop: state interface echoes commanded position (virtual encoder).
      Hardware: Arduino Uno R4 WiFi (RA4M1) + 3-Axis CNC Shield + 3x A4988 + NEMA 17.
      Transmission: S2M timing belt, 3.8:1 gear ratio (from Parmiggiani et al.).
    </description>
  </class>
</library>
```

**URDF `ros2_control` tag for real hardware (in `deltaflex.ros2_control.xacro`):**

```xml
<ros2_control name="DeltaFlexHardware" type="system">
  <hardware>
    <plugin>deltaflex_hardware/DeltaFlexHardwareInterface</plugin>
    <param name="serial_port">/dev/ttyACM0</param>
    <param name="baud_rate">115200</param>
    <param name="steps_per_rev">1600</param>   <!-- 200 × 8 microsteps (1/8 mode); gear_ratio=3.8 applied in GRBL $100 setting -->
    <param name="feed_rate">60.0</param>        <!-- rad/min = 1 rad/s -->
  </hardware>

  <joint name="joint_1">
    <command_interface name="position"/>
    <state_interface name="position"/>
  </joint>
  <joint name="joint_2">
    <command_interface name="position"/>
    <state_interface name="position"/>
  </joint>
  <joint name="joint_3">
    <command_interface name="position"/>
    <state_interface name="position"/>
  </joint>
</ros2_control>
```

#### 7.4.1 Alternative: Custom Protocol for Uno R4 WiFi (Option A firmware)

The `write()` function below replaces the G-code variant when using the custom serial firmware. Steps are absolute; gear ratio and microstepping are baked in at initialisation.

```cpp
// write() variant for custom firmware (Uno R4 WiFi + AccelStepper)
// Steps are absolute; gear ratio and microstepping baked in at init.
hardware_interface::return_type DeltaFlexHardwareInterface::write(
    const rclcpp::Time &, const rclcpp::Duration &)
{
  const double GEAR_RATIO = 3.8;
  std::string cmd = "M";
  for (size_t i = 0; i < 3; i++) {
    long steps = static_cast<long>(
        (hw_commands_[i] / (2.0 * M_PI)) * steps_per_rev_ * GEAR_RATIO);
    cmd += std::to_string(steps);
    if (i < 2) cmd += " ";
    hw_states_[i] = hw_commands_[i];  // virtual encoder
  }
  cmd += "\n";
  send_cmd(cmd);       // blocking write to serial
  wait_for_ok();       // waits for "ok\n" response
  return hardware_interface::return_type::OK;
}
```

---

### 7.5 CMakeLists.txt additions

Add the following to `deltaflex_hardware/CMakeLists.txt` to build the hardware interface as a pluginlib plugin:

```cmake
cmake_minimum_required(VERSION 3.8)
project(deltaflex_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Find dependencies ────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# ── Build the hardware interface shared library ──────────────────────────────
add_library(${PROJECT_NAME} SHARED
  src/deltaflex_hardware_interface.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# ── Export plugin description XML ────────────────────────────────────────────
pluginlib_export_plugin_description_file(hardware_interface deltaflex_hardware.xml)

# ── Install ──────────────────────────────────────────────────────────────────
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES deltaflex_hardware.xml
  DESTINATION share/${PROJECT_NAME}
)

# ── ament package ────────────────────────────────────────────────────────────
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)
ament_package()
```

---

### 7.6 Testing the connection without ROS2

Before integrating ROS2, verify that GRBL responds correctly over serial. This step is essential for debugging wiring and configuration issues without the complexity of the full ROS2 stack.

```bash
# Install minicom
sudo apt install minicom

# Connect to GRBL (press Ctrl-A then Z for minicom help, Ctrl-A X to quit)
minicom -D /dev/ttyACM0 -b 115200
```

Alternatively, use `screen`:

```bash
screen /dev/ttyACM0 115200
# Quit with: Ctrl-A then k (kill), confirm y
```

Inside the terminal session, type these commands and verify the responses:

```
# Query all GRBL settings
$$
# GRBL prints all $N=value lines, ending with 'ok'

# Query current status
?
# GRBL responds: <Idle|MPos:0.000,0.000,0.000|FS:0,0>

# Move joint_1 to 0.5 rad (X axis), keep Y and Z at 0
G90 G1 X0.5 Y0.0 Z0.0 F60
# GRBL responds: ok
# Motor 1 (joint_1) should rotate

# Move all joints to 0.5 rad simultaneously
G90 G1 X0.5 Y0.5 Z0.5 F60
# GRBL responds: ok
# All three NEMA 17s should move together

# Return all joints to zero (home position)
G90 G1 X0.0 Y0.0 Z0.0 F60
# GRBL responds: ok

# Confirm GRBL reached idle state
?
# Expected: <Idle|MPos:0.500,0.500,0.500|FS:0,0>
# (MPos reflects the commanded position in GRBL coordinates)
```

**What to check if motors do not move:**
1. Verify the ENABLE pin (D8) is being held LOW — jumper the ENABLE pin to GND on the CNC shield if testing without the full firmware enable.
2. Check that 12V power is connected to the CNC shield's V+ and GND terminals (not just USB power — the A4988 motor outputs need the 12V supply).
3. Confirm `$100`–`$102` show `967.7` (1/8 microstepping, 3.8:1 gear ratio) via `$$` output.
4. Use a multimeter on the STEP pin (D2) to verify pulses during a move command.

**Limit switches (Omron D2MQ-4L-105-1, one per axis):**

The paper (Parmiggiani et al.) specifies Omron D2MQ-4L-105-1 limit switches for homing, one per axis. Connect them to the CNC Shield limit switch pins:

```
Limit switch connections (CNC Shield v3):
  X axis (joint_1): X_MIN pin = D9
  Y axis (joint_2): Y_MIN pin = D10
  Z axis (joint_3): Z_MIN pin = D11
  Common (GND):     GND rail on CNC shield

Switch wiring: normally open (NO) between signal pin and GND.
When triggered: pin pulled LOW → GRBL registers home position.
```

Enable GRBL homing with:

```
$21=1     # Enable hard limits (limit switches as safety stops)
$22=1     # Enable homing cycle
$23=0     # Homing direction mask (0 = home toward negative direction for all axes)
$25=100   # Homing seek rate [rad/min] (slow approach for accuracy)
$26=250   # Homing debounce delay [ms]
$27=0.1   # Homing pull-off distance [rad] (back off after triggering switch)
```

After wiring limit switches, trigger a homing cycle:

```
# In minicom / serial terminal:
$H        # Run homing cycle (all axes home simultaneously by default)
          # GRBL moves each axis until limit switch triggers, then backs off
          # After completion: GRBL coordinate origin = home position
?
          # Expected: <Idle|MPos:0.000,0.000,0.000|FS:0,0>
```

> **Note:** The `on_activate()` method in the hardware interface sends `G92 X0 Y0 Z0` to zero GRBL’s work coordinates at the current position. If GRBL homing (`$H`) has already run, the machine coordinates are already correct and `G92` may conflict. For homing-enabled setups, remove the `G92` reset from `on_activate()` and rely on `$H` instead.


**Testing with the custom serial firmware (Uno R4 WiFi, Option A):**

For the Uno R4 WiFi running the custom `AccelStepper` firmware, test with:

```bash
minicom -D /dev/ttyACM0 -b 115200
# Then type:
?            # query current position → X0 Y0 Z0
HOME         # set zero position
M484 484 484 # move all joints to ~0.5 rad (1600 steps/rev * 3.8 * 0.5/(2π) ≈ 484)
?            # confirm position updated
```

---

### 7.7 real.launch.py

The real hardware launch file starts `ros2_control_node` with the custom GRBL hardware interface. Gazebo is not started.

**File: `deltaflex_bringup/launch/real.launch.py`**

```python
"""
real.launch.py

Starts real hardware bringup for DeltaFlex with Arduino Uno + CNC Shield v3 + GRBL.

Nodes started:
  1. robot_state_publisher  — publishes /robot_description and TF
  2. ros2_control_node      — controller_manager with DeltaFlexHardwareInterface
  3. joint_state_broadcaster — publishes /joint_states
  4. deltaflex_position_controller — ForwardCommandController on 3 joints
  5. ik_service_node        — IK/FK ROS2 services
  6. cartesian_controller   — Cartesian-space command node

Arguments:
  serial_port   (default: /dev/ttyACM0)  — serial port for GRBL
  baud_rate     (default: 115200)        — GRBL baud rate
  steps_per_rev (default: 1600)          — 200 steps × 8 microsteps
  feed_rate     (default: 60.0)          — rad/min (= 1 rad/s)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
)
from launch_ros.actions import Node


def generate_launch_description():

    # ── Paths ────────────────────────────────────────────────────────────────
    description_pkg = get_package_share_directory('deltaflex_description')
    bringup_pkg     = get_package_share_directory('deltaflex_bringup')

    xacro_file     = os.path.join(description_pkg, 'urdf', 'deltaflex.urdf.xacro')
    controllers_yaml = os.path.join(bringup_pkg, 'config', 'controllers.yaml')

    # ── Arguments ────────────────────────────────────────────────────────────
    declare_serial_port = DeclareLaunchArgument(
        'serial_port', default_value='/dev/ttyACM0',
        description='Serial port for GRBL (e.g. /dev/ttyACM0 or /dev/ttyUSB0)'
    )
    declare_baud_rate = DeclareLaunchArgument(
        'baud_rate', default_value='115200',
        description='GRBL baud rate'
    )
    declare_steps_per_rev = DeclareLaunchArgument(
        'steps_per_rev', default_value='1600',
        description='Steps per revolution (200 * microstep_divisor)'
    )
    declare_feed_rate = DeclareLaunchArgument(
        'feed_rate', default_value='60.0',
        description='GRBL feed rate in rad/min (default 60 = 1 rad/s)'
    )

    serial_port   = LaunchConfiguration('serial_port')
    baud_rate     = LaunchConfiguration('baud_rate')
    steps_per_rev = LaunchConfiguration('steps_per_rev')
    feed_rate     = LaunchConfiguration('feed_rate')

    # ── Robot description — real hardware (use_sim:=false) ───────────────────
    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ',
        xacro_file,
        ' use_sim:=false',
        ' serial_port:=', serial_port,
        ' baud_rate:=',   baud_rate,
        ' steps_per_rev:=', steps_per_rev,
        ' feed_rate:=',   feed_rate,
    ])
    robot_description = {'robot_description': robot_description_content}

    # ── robot_state_publisher ────────────────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    # ── ros2_control_node (controller manager + hardware interface) ──────────
    # This node loads the DeltaFlexHardwareInterface plugin, opens the serial
    # port to GRBL, and runs the ros2_control control loop at update_rate Hz.
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            robot_description,
            controllers_yaml,
        ],
        output='screen',
    )

    # ── Activate controllers after ros2_control_node starts ──────────────────
    activate_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    activate_pos_ctrl = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['deltaflex_position_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # Start position controller after joint_state_broadcaster is up
    activate_pos_ctrl_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=activate_jsb,
            on_exit=[activate_pos_ctrl],
        )
    )

    # ── IK service node ───────────────────────────────────────────────────────
    ik_node = Node(
        package='deltaflex_kinematics',
        executable='ik_service_node.py',
        name='deltaflex_ik_service',
        output='screen',
    )

    # ── Cartesian controller ──────────────────────────────────────────────────
    cartesian_node = Node(
        package='deltaflex_controllers',
        executable='cartesian_controller_node.py',
        name='deltaflex_cartesian_controller',
        output='screen',
    )

    return LaunchDescription([
        declare_serial_port,
        declare_baud_rate,
        declare_steps_per_rev,
        declare_feed_rate,
        robot_state_publisher,
        ros2_control_node,
        activate_jsb,
        activate_pos_ctrl_after_jsb,
        ik_node,
        cartesian_node,
    ])
```

**Usage:**

```bash
# Default (Arduino Uno on /dev/ttyACM0, 1/8 microstepping)
ros2 launch deltaflex_bringup real.launch.py

# Override port if Uno appears as ttyUSB0 (e.g., CH340 clone)
ros2 launch deltaflex_bringup real.launch.py serial_port:=/dev/ttyUSB0

# Override for 1/16 microstepping (all 3 jumpers shorted)
ros2 launch deltaflex_bringup real.launch.py steps_per_rev:=3200
```

---

### 7.8 Wiring diagram

```
  ┌──────────────────────────────────────────────────────────────────┐
  │  12V DC Power Supply                                             │
  │  (2A min per axis → 6A+ recommended for 3x NEMA 17)             │
  └──────┬──────────────────────────────────────────────────────────┘
         │  12V+                              GND
         │                                    │
  ┌──────▼──────────────────────────────────────────────────────────┐
  │  3-Axis CNC/Stepper Motor Shield (stacked on Arduino UNO WiFi)  │
  │                                                                  │
  │  V+●──── 12V from PSU        GND●──── GND from PSU             │
  │                                                                  │
  │  ┌─────────────────────────────────────────────────────────┐    │
  │  │  X slot          Y slot          Z slot                 │    │
  │  │  ┌────────┐      ┌────────┐      ┌────────┐            │    │
  │  │  │ A4988  │      │ A4988  │      │ A4988  │            │    │
  │  │  │(joint1)│      │(joint2)│      │(joint3)│            │    │
  │  │  └───┬────┘      └───┬────┘      └───┬────┘            │    │
  │  │      │               │               │                  │    │
  │  │  MS1─┤ ●shorted   MS1┤ ●shorted  MS1┤ ●shorted        │    │
  │  │  MS2─┤ ●shorted   MS2┤ ●shorted  MS2┤ ●shorted        │    │
  │  │  MS3─┤ ○open      MS3┤ ○open     MS3┤ ○open           │    │
  │  │      │  (→1/8 mode)  │               │                  │    │
  │  │      │  (MS1+MS2 shorted, MS3 open = 1/8 = 1600 spr)   │    │
  │  │      │  (Short all 3 for 1/16 = 3200 spr)              │    │
  │  └──────┼───────────────┼───────────────┼──────────────────┘    │
  │         │               │               │                        │
  │  1A●  2A●  3A●  4A●  1B●  2B●  ...motor outputs (x3)           │
  │                                                                  │
  │  Arduino UNO WiFi Rev.2 header (underneath shield)               │
  │  D2=STEP_X  D5=DIR_X                                            │
  │  D3=STEP_Y  D6=DIR_Y                                            │
  │  D4=STEP_Z  D7=DIR_Z                                            │
  │  D8=ENABLE (active LOW — shield pulls LOW when powered)          │
  └─────────────────────────────────────────────────────────────────┘
         │ USB-B cable
         ▼
  ┌──────────────────────┐
  │  Arduino UNO WiFi    │
  │  R4 WiFi (RA4M1)     │
  │  custom firmware     │
  └──────────┬───────────┘
             │ USB cable (/dev/ttyACM0)
             ▼
  ┌──────────────────────────────────────────────────────┐
  │  Linux PC (Ubuntu 24.04)                             │
  │  ROS2 Jazzy                                          │
  │  deltaflex_hardware/DeltaFlexHardwareInterface       │
  │  → ros2_control → /deltaflex_position_controller    │
  └──────────────────────────────────────────────────────┘

  Motor connections (3× NEMA 17, identical for each axis):
  ┌─────────────────────────────────────────────────┐
  │  A4988 output pins → NEMA 17 coil wires         │
  │  1A, 2A ──── Coil A  (e.g. red + blue wires)   │
  │  1B, 2B ──── Coil B  (e.g. green + black wires) │
  │                                                  │
  │  Verify coil pairs with multimeter:              │
  │  Coil A: continuity between 1A↔2A               │
  │  Coil B: continuity between 1B↔2B               │
  │  No continuity between A and B coils             │
  └─────────────────────────────────────────────────┘

  Current limit: Set Vref on each A4988 trim pot.
  DeltaFlex motor winding: 2.80 V rated. Safe operating current: ~1.5 A.
  A4988 Vref formula (Rsense=0.1Ω): Vref = I_max × 0.625
  For 1.5 A: Vref = 1.5 × 0.625 = 0.9375 V ≈ 0.94 V  [RECOMMENDED for DeltaFlex]
  Start conservatively at Vref=0.5V (≈0.8A) and increase to 0.94V while checking motor heat.
  Measure Vref between the trim pot center pin and GND.
  Note: the 3.8:1 gear ratio means the motor turns faster than the joint;
  adequate current is critical to prevent stalling during coordinated 3-axis moves.
```

---

## 8. Cartesian Control Node

### File: `deltaflex_controllers/scripts/cartesian_controller_node.py`

```python
#!/usr/bin/env python3
"""
Cartesian control node for DeltaFlex.

Subscribes:
  /delta/target_pose  (geometry_msgs/Point)
    Desired end-effector position in robot base frame [m].

Publishes:
  /deltaflex_position_controller/commands  (std_msgs/Float64MultiArray)
    Motor joint angles [theta_1, theta_2, theta_3] in radians.

Parameters:
  interpolation_steps: int    (default: 50)  Steps for linear interpolation
  interpolation_dt:    float  (default: 0.02) Time between steps [s]
  workspace_r_max:     float  (default: 0.05) Max radial offset [m]
  workspace_z_min:     float  (default: 0.15) Min Z depth [m]
  workspace_z_max:     float  (default: 0.30) Max Z depth [m]
"""

import rclpy
from rclpy.node import Node
import numpy as np
import threading

from geometry_msgs.msg import Point
from std_msgs.msg import Float64MultiArray

from deltaflex_kinematics.delta_ik import (
    inverse_kinematics, workspace_check
)


class CartesianController(Node):
    def __init__(self):
        super().__init__('deltaflex_cartesian_controller')

        # Parameters
        self.declare_parameter('interpolation_steps', 50)
        self.declare_parameter('interpolation_dt',    0.02)
        self.declare_parameter('workspace_r_max',     0.05)
        self.declare_parameter('workspace_z_min',     0.15)
        self.declare_parameter('workspace_z_max',     0.30)

        self._interp_steps  = self.get_parameter('interpolation_steps').value
        self._interp_dt     = self.get_parameter('interpolation_dt').value
        self._ws_r_max      = self.get_parameter('workspace_r_max').value
        self._ws_z_min      = self.get_parameter('workspace_z_min').value
        self._ws_z_max      = self.get_parameter('workspace_z_max').value

        # Home position (EE at geometric centre, approximate)
        self._current_pos = np.array([0.0, 0.0, 0.22])  # metres
        self._current_joints = inverse_kinematics(*self._current_pos)

        # Subscriber
        self._target_sub = self.create_subscription(
            Point,
            '/delta/target_pose',
            self._target_callback,
            10
        )

        # Publisher
        self._cmd_pub = self.create_publisher(
            Float64MultiArray,
            '/deltaflex_position_controller/commands',
            10
        )

        # Motion lock — prevent overlapping trajectories
        self._motion_lock = threading.Lock()

        self.get_logger().info('Cartesian controller ready.')
        self._publish_joints(self._current_joints)

    def _target_callback(self, msg: Point):
        target = np.array([msg.x, msg.y, msg.z])

        # Workspace check
        if not workspace_check(
            target[0], target[1], target[2],
            r_max=self._ws_r_max,
            z_min=self._ws_z_min,
            z_max=self._ws_z_max
        ):
            self.get_logger().warn(
                f'Target ({target[0]:.4f}, {target[1]:.4f}, {target[2]:.4f}) '
                f'outside workspace — ignored.'
            )
            return

        # IK check at target
        try:
            target_joints = inverse_kinematics(*target)
        except ValueError as e:
            self.get_logger().warn(f'IK failed for target: {e}')
            return

        # Run interpolated motion in a separate thread so the subscriber
        # callback returns immediately
        thread = threading.Thread(
            target=self._execute_linear_move,
            args=(self._current_pos.copy(), target, target_joints),
            daemon=True
        )
        thread.start()

    def _execute_linear_move(self, start: np.ndarray,
                              target: np.ndarray,
                              target_joints: np.ndarray):
        """
        Execute a Cartesian-space linear interpolation from start to target.
        Each intermediate waypoint is solved with IK.
        """
        if not self._motion_lock.acquire(blocking=False):
            self.get_logger().warn('Motion already in progress — ignoring command.')
            return

        try:
            import time
            n = self._interp_steps

            for i in range(1, n + 1):
                alpha = i / n
                waypoint = start + alpha * (target - start)

                try:
                    joints = inverse_kinematics(*waypoint)
                except ValueError:
                    self.get_logger().warn(
                        f'IK failed at interpolation step {i}/{n} — stopping.')
                    break

                self._publish_joints(joints)
                time.sleep(self._interp_dt)

            # Update current position
            self._current_pos    = target.copy()
            self._current_joints = target_joints.copy()

        finally:
            self._motion_lock.release()

    def _publish_joints(self, joints: np.ndarray):
        msg = Float64MultiArray()
        msg.data = joints.tolist()
        self._cmd_pub.publish(msg)

    def _go_home(self):
        """Send robot to home position (0, 0, z_home)."""
        home = np.array([0.0, 0.0, 0.22])
        try:
            joints = inverse_kinematics(*home)
            self._execute_linear_move(
                self._current_pos.copy(), home, joints)
        except ValueError as e:
            self.get_logger().error(f'Cannot reach home: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = CartesianController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Testing with a simple command

```bash
# Move EE to (x=0, y=0, z=0.22) — home
ros2 topic pub --once /delta/target_pose geometry_msgs/msg/Point \
  '{x: 0.0, y: 0.0, z: 0.22}'

# Move EE 20 mm in X direction
ros2 topic pub --once /delta/target_pose geometry_msgs/msg/Point \
  '{x: 0.02, y: 0.0, z: 0.22}'

# Or publish joint angles directly (bypass IK, for debugging):
ros2 topic pub --once /deltaflex_position_controller/commands \
  std_msgs/msg/Float64MultiArray '{data: [0.1, 0.1, 0.1]}'
```

---

## 9. MoveIt2 Integration

MoveIt2 supports delta robots but requires a custom kinematics plugin because the default `KDLKinematicsPlugin` solves serial chain kinematics using the Jacobian — it has no concept of a parallel mechanism and will produce incorrect results.

### Recommended approach: `bio_ik`

[`bio_ik`](https://github.com/TAMS-Group/bio_ik) is a gradient-free numerical IK solver that works by minimising a cost function over joint space. For a delta robot:

1. Define a `PoseGoal` (desired EE pose).
2. `bio_ik` searches joint space using evolutionary optimisation.
3. Because the URDF only models the upper arms as actuated joints, `bio_ik` will find the three motor angles that minimise the distance between the forward-kinematics EE frame and the target — effectively solving the delta IK numerically.

```yaml
# In your MoveIt2 kinematics.yaml:
deltaflex:
  kinematics_solver: bio_ik/BioIKKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.005
  kinematics_solver_attempts: 3
```

### Custom plugin approach

For production use, implement `moveit_core/kinematics_base/KinematicsBase` and delegate to your `delta_ik.py` logic (exposed via the IK service, or wrapped in a C++ shared library). This gives deterministic, fast IK without the search overhead of `bio_ik`.

MoveIt2 is most useful for DeltaFlex if you need:
- Collision-aware path planning (pick-and-place with obstacles)
- Cartesian path planning (`moveit_msgs/CartesianPath`)
- Integration with perception (point cloud collision maps)

For simple point-to-point Cartesian moves, the `CartesianController` node in Section 8 is sufficient and has far lower overhead.

---

## 10. Launch File Overview

```
deltaflex_ros2/
├── deltaflex_description/launch/
│   └── view_robot.launch.py
│       Starts: robot_state_publisher, joint_state_publisher_gui, RVIZ2
│       Use:    Visualize URDF before simulation (no Gazebo)
│
├── deltaflex_bringup/launch/
│   ├── sim.launch.py
│   │   Starts: gz sim (Harmonic), robot_state_publisher,
│   │           ros_gz_sim/create (spawn), gz_clock_bridge,
│   │           [gz_ros2_control plugin starts controller_manager],
│   │           joint_state_broadcaster, deltaflex_position_controller,
│   │           ik_service_node
│   │   Args:   gui:=true/false, verbose:=true/false
│   │
│   └── real.launch.py
│       Starts: robot_state_publisher,
│               ros2_control_node (DeltaFlexHardwareInterface → GRBL/serial),
│               joint_state_broadcaster, deltaflex_position_controller,
│               ik_service_node, cartesian_controller_node
│       Args:   serial_port:=/dev/ttyACM0, baud_rate:=115200,
│               steps_per_rev:=1600, feed_rate:=60.0
```

---

## 11. Step-by-Step Getting Started

### Step 1 — Install dependencies

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-gz-ros2-control-demos \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-xacro \
  ros-jazzy-rviz2 \
  python3-numpy \
  avrdude \
  minicom
```

> **Note:** `ros-jazzy-gz-ros2-control` installs `libgz_ros2_control-system.so` and the `GazeboSimSystem` hardware plugin. Gazebo Harmonic is the officially supported Gazebo version for ROS2 Jazzy — confirmed in the [Gazebo/ROS compatibility matrix](https://gazebosim.org/docs/latest/ros_installation/).

### Step 2 — Create the workspace and packages

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Create the package tree
ros2 pkg create deltaflex_description \
  --build-type ament_cmake \
  --dependencies urdf xacro

ros2 pkg create deltaflex_bringup \
  --build-type ament_cmake \
  --dependencies launch launch_ros

ros2 pkg create deltaflex_kinematics \
  --build-type ament_python \
  --dependencies rclpy std_msgs geometry_msgs

ros2 pkg create deltaflex_controllers \
  --build-type ament_python \
  --dependencies rclpy std_msgs geometry_msgs

ros2 pkg create deltaflex_hardware \
  --build-type ament_cmake \
  --dependencies hardware_interface pluginlib rclcpp
```

Populate with the files from this guide.

### Step 3 — Export meshes from the STEP files

The DeltaFlex STEP files are in [the repository's `hardware/` directory](https://github.com/made-iit/deltaflex/tree/main/hardware). Export each component as a STL file:

```bash
# Using FreeCAD CLI (headless):
freecadcmd -c "
import FreeCAD, Mesh
doc = FreeCAD.openDocument('deltaflex_upper_arm.step')
for obj in doc.Objects:
    Mesh.export([obj], obj.Label + '.stl')
"
```

Place exported STLs in `deltaflex_description/meshes/`. Update the URDF `<mesh filename="package://deltaflex_description/meshes/..."/>` references.

### Step 4 — Measure kinematic parameters

Open the STEP files in FreeCAD or your preferred CAD tool and measure:
- `F`: distance from base centre to motor pivot (circumradius of base equilateral triangle)
- `E`: distance from EE centre to forearm attachment point
- `RF`: upper arm length (motor pulley axis to forearm spherical joint)
- `RE`: forearm length (upper arm spherical joint to EE spherical joint)

Update these values in `deltaflex_kinematics/deltaflex_kinematics/delta_ik.py`.

### Step 5 — Build

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select \
  deltaflex_description \
  deltaflex_bringup \
  deltaflex_kinematics \
  deltaflex_controllers \
  deltaflex_hardware

source install/setup.bash
```

### Step 6 — Verify URDF

```bash
# Check for URDF parse errors
ros2 run xacro xacro \
  src/deltaflex_description/urdf/deltaflex.urdf.xacro use_sim:=true \
  | ros2 run urdfdom check_urdf /dev/stdin

# Visualize in RVIZ2 (no Gazebo)
ros2 launch deltaflex_description view_robot.launch.py
```

### Step 7 — Flash GRBL and configure

Flash GRBL 1.1 to the Arduino Uno (see Section 7.2), then apply the `$`-settings from Section 7.3. Verify the serial connection using `minicom` as described in Section 7.6 before proceeding.

```bash
# Flash GRBL
wget https://github.com/gnea/grbl/releases/download/v1.1h.20190825/grbl_v1.1h.20190825.hex
avrdude -v -p atmega328p -c arduino -P /dev/ttyACM0 -b 115200 \
        -D -U flash:w:grbl_v1.1h.20190825.hex:i

# Verify serial connection
minicom -D /dev/ttyACM0 -b 115200
# Type: $$ (should list settings), then: G90 G1 X0.5 F60 (should move joint_1)
```

### Step 8 — Launch simulation

```bash
ros2 launch deltaflex_bringup sim.launch.py
```

Wait for Gazebo to load (~5–10 s). The robot should appear suspended above the ground.

### Step 9 — Verify controllers are active

```bash
ros2 control list_controllers
# Expected:
# joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
# deltaflex_position_controller[forward_command_controller/ForwardCommandController] active
```

### Step 10 — Send a test joint command

```bash
# Command all joints to 0.3 rad
ros2 topic pub --once /deltaflex_position_controller/commands \
  std_msgs/msg/Float64MultiArray '{data: [0.3, 0.3, 0.3]}'

# Watch joint states
ros2 topic echo /joint_states
```

### Step 11 — Test the IK service

```bash
# Call IK service directly
ros2 service call /deltaflex/ik deltaflex_kinematics/srv/IKSolve \
  '{x: 0.0, y: 0.0, z: 0.22}'

# Expected response (approximate, depends on your geometry parameters):
# joint_angles: [0.0, 0.0, 0.0]
# success: True
```

### Step 12 — Send a Cartesian command

```bash
# Start cartesian controller node (if not already in launch file)
ros2 run deltaflex_controllers cartesian_controller_node.py

# Send a target pose
ros2 topic pub --once /delta/target_pose geometry_msgs/msg/Point \
  '{x: 0.02, y: 0.01, z: 0.23}'
```

### Step 13 — Launch real hardware

Once simulation is verified, bring up real hardware:

```bash
ros2 launch deltaflex_bringup real.launch.py

# Override port if needed
ros2 launch deltaflex_bringup real.launch.py serial_port:=/dev/ttyUSB0
```

---

## 12. Key Challenges & Tips

### Compliant joint behavior in Gazebo

The DeltaFlex's monolithic SLA-printed flexure joints act as torsional springs, not rigid revolute joints. In Gazebo simulation:

- **Simple approach (recommended for control development):** Treat all joints as rigid. The open-chain URDF already does this. The compliant behavior only matters for dynamics/vibration studies.
- **Advanced approach:** Use `gz-sim-spring-joint-system` or a custom `GazeboSimSystemInterface` that adds torsional spring torque as a function of joint deflection: \(\tau = -k \cdot \theta\). The DeltaFlex paper (Parmiggiani et al.) provides measured stiffness values: **0.41 N/mm** (perpendicular/z-axis) and **0.19 N/mm** (tangential/x-axis). These are translational stiffness values from Cartesian force tests; convert to torsional stiffness using the arm geometry for use as Gazebo spring constants.
- **FEM verification:** The paper uses FEM for structural verification. You can import the deformed mesh as a Gazebo visual for qualitative demonstration, but it won't affect dynamics.

### Calibration and home position

**Home position convention:** Define home as all three motor angles at θ = 0, with the upper arms horizontal. The end-effector will be at the lowest reachable point directly below the base centre. This must match your IK model.

**Limit switches:** The paper (Parmiggiani et al.) uses **Omron D2MQ-4L-105-1** microswitches (one per axis) for automated homing. With limit switches wired to CNC Shield pins D9/D10/D11:

```
# Enable and run GRBL homing cycle:
$22=1     # Enable homing cycle
$23=0     # Homing direction mask (0 = home toward negative end for all axes)
$H        # Execute homing cycle
          # Each axis moves to its limit switch, triggers, backs off by $27, then zeros
```

**Physical homing procedure (real hardware with GRBL, without limit switches):**
1. Power on with steppers disabled (GRBL alarm state after reset is normal).
2. Send `$X` to GRBL to unlock from alarm state.
3. Manually jog each axis to the home position using `G91 G1 X<delta> F30` (relative moves).
4. Zero GRBL's work coordinates at the current position: `G92 X0 Y0 Z0`.
5. The hardware interface sends `G92 X0 Y0 Z0` automatically on `on_activate()`, so always position the robot at home before launching `real.launch.py`.

```bash
# Manual homing via minicom before launching ROS2:
# 1. Unlock GRBL
$X
# 2. Jog to home position (relative mode, small increments)
G91 G1 X0.1 F30    # nudge joint_1 positive 0.1 rad
G91 G1 X-0.1 F30   # nudge back
# 3. Once at home, zero coordinates
G92 X0 Y0 Z0
# 4. Return to absolute mode
G90
```

> **Stiffness warning:** DeltaFlex stiffness is intentionally low (0.19–0.41 N/mm). The robot complies easily under load. Do not overshoot workspace limits (±50 mm XY, ±60 mm Z) — the Duraform PA flexure joints (0.7 mm thick) can deform plastically at extreme angles.

### Workspace limits

From the repeatability experiment (Fig. 7, page 8/12 of Parmiggiani et al.), the 6 reference configurations tested define the practical workspace boundary:

| Dimension | Value (from paper) | Notes |
|-----------|--------------------|-------|
| XY radial range | **±50 mm** | Confirmed from repeatability test |
| Z range | **±60 mm** | Confirmed from repeatability test |
| Maximum motor angle | ±45° (π/4 rad) | Approximate; measure from STEP file |

These values are implemented as defaults in `workspace_check()` (see Section 4). The compliant flexure joints limit joint travel to less than a rigid pivot — the robot’s low stiffness (0.19–0.41 N/mm) means it complies easily near workspace boundaries. Do not command positions outside these limits.

### Stepper timing with Arduino Uno R4 WiFi (RA4M1)

- The Uno R4 WiFi uses the Renesas RA4M1 (ARM Cortex-M4); flash using arduino-cli (`arduino:renesas_uno:unor4wifi` fqbn), not avrdude. The custom AccelStepper firmware (Option A) manages all step timing in software on the RA4M1 — the host PC sends `M<j1> <j2> <j3>` commands and waits for `ok` responses.
- **3.8:1 gear ratio impact:** The motor turns 3.8× faster than the arm joint. At a joint velocity of 1 rad/s with 1/8 microstepping, the motor generates 1600 × 3.8 ÷ (2π) ≈ 968 steps/sec per axis. GRBL can handle this easily; the 3.8:1 ratio is beneficial — it reduces reflected inertia and increases effective holding torque at the joint to ~0.59 × 3.8 = **~2.2 Nm**.
- At 1/8 microstepping (1600 steps/rev), GRBL can comfortably generate step pulses for all three axes simultaneously at up to ~30,000 steps/sec total, well above the DeltaFlex's operational range.
- If you observe stalling or missed steps at moderate speeds, reduce `$110`–`$112` (max rate) and `$120`–`$122` (acceleration) in the GRBL settings, and verify the A4988 Vref is set correctly.
- **A4988 Vref setting for the DeltaFlex motor (2.80 V rated winding):** The motor’s rated current must not be exceeded. For a safe operating current of ~1.5 A:

  \[
  V_{\text{ref}} = I_{\text{max}} \times 0.625 = 1.5 \times 0.625 = 0.9375 \approx 0.94 \text{ V}
  \]

  Measure Vref between the A4988 trim pot centre pin and GND. Start at 0.5 V and increase while checking for motor heat. The formula assumes `Rsense = 0.1 Ω` (standard A4988 module). The general formula is `I_max = Vref / 0.625`.
- The limiting factor is typically mechanical (motor current, load inertia) rather than computational.

### GRBL serial protocol robustness

The hardware interface's `wait_for_ok()` method implements basic request/response synchronization. GRBL has a 15-line receive buffer; as long as `write()` is called at the `ros2_control` update rate (100 Hz) and waits for `ok` before returning, buffer overflow is not a concern. For production hardening:

- Monitor for GRBL `error:` responses and surface them as `RCLCPP_ERROR` messages.
- Implement a watchdog in `read()` that checks GRBL status (`?` query) if no `ok` has been received within a timeout window.
- Add a serial reconnect path in `on_activate()` for USB hot-plug recovery.

### URDF-to-SDF conversion quirks

Gazebo Harmonic internally converts URDF to SDFormat. During this conversion:
- `<fixed>` joints between links with negligible mass may be merged (link lumping). Disable this with `<preserve_fixed_joint>true</preserve_fixed_joint>` inside the `<gazebo>` tag for that joint.
- The `<gazebo reference="link_name">` tag lets you add Gazebo-specific properties (friction, material, sensors) without polluting the URDF.

```xml
<gazebo reference="upper_arm_1">
  <mu1>0.2</mu1>
  <mu2>0.2</mu2>
  <kp>1e6</kp>
  <kd>100</kd>
</gazebo>
```

### Debugging tools

```bash
# Inspect the full controller manager state
ros2 control list_hardware_interfaces

# Publish a single joint command and watch /joint_states in RVIZ2
ros2 topic pub -r 10 /deltaflex_position_controller/commands \
  std_msgs/msg/Float64MultiArray '{data: [0.1, 0.1, 0.1]}'

# Plot joint state vs. command with plotjuggler
ros2 run plotjuggler plotjuggler

# Check transform tree
ros2 run tf2_tools view_frames
```

---

## References & Sources

- [DeltaFlex repository (made-iit/deltaflex)](https://github.com/made-iit/deltaflex)
- [DeltaFlex project page & paper abstract](https://made-iit.github.io/deltaflex/)
- [gz_ros2_control official documentation (ROS2 Jazzy)](https://control.ros.org/jazzy/doc/gz_ros2_control/doc/index.html)
- [gz_ros2_control GitHub repository](https://github.com/ros-controls/gz_ros2_control)
- [ros2_control controllers index (Jazzy)](https://control.ros.org/jazzy/doc/ros2_controllers/doc/controllers_index.html)
- [ros2_control hardware interface types](https://control.ros.org/rolling/doc/ros2_control/hardware_interface/doc/hardware_interface_types_userdoc.html)
- [Gazebo + ROS installation / compatibility matrix](https://gazebosim.org/docs/latest/ros_installation/)
- [GRBL 1.1 firmware (gnea/grbl)](https://github.com/gnea/grbl)
- [CNC Shield v3 documentation and pinout](https://blog.protoneer.co.nz/arduino-cnc-shield/)
- [A4988 stepper driver datasheet (Pololu)](https://www.pololu.com/product/1182)
- [Custom ros2_control hardware interface (Arduino example)](https://github.com/masum919/ros2_control_custom_hardware_interface)
- [bio_ik MoveIt2 kinematics plugin](https://github.com/TAMS-Group/bio_ik)
