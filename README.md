
# Custom IK

A small C++ project for learning kinematics and inverse kinematics

## What this repo is for

- Models a simple 2D/3D robot arm as a chain of joints and transforms
- Computes forward kinematics by composing transforms
- Builds a Jacobian from the arm geometry
- Solves IK using a damped least-squares update
- Includes a minimal ROS 2 / MoveIt package boundary for the PX150 robot model

## Core mathematics

### 1. Transforms
Each joint stores a local transform made from:
- an offset translation/rotation
- a rotation about one axis: X, Y, or Z

The local transform for a joint is:

$$T_{local} = T_{offset} \cdot R(axis, \theta)$$

The full pose of the end-effector is obtained by chaining transforms along the robot:

$$T_{ee} = T_{base} \cdot T_1 \cdot T_2 \cdots T_n$$

### 2. Jacobian
A Jacobian maps small changes in joint angles to small changes in end-effector motion:

$$\delta x = J \delta \theta$$

In this project, the Jacobian is built directly from the arm geometry:

$$J_{v_i} = z_i \times (p_e - p_i)$$

$$J_{w_i} = z_i$$

### 3. Inverse kinematics
The solver uses a damped least-squares update:

$$\delta \theta = J^T (J J^T + \lambda I)^{-1} e$$

where $e$ is the position/orientation error between the target pose and the current pose.

## Project structure

- [main.cpp](main.cpp) – the single educational walkthrough entry point
- [math/](math/) – vector, matrix, transform, quaternion, and Jacobian helpers
- [kinematics/forward_kinematics/](kinematics/forward_kinematics/) – joint and chain definitions
- [solvers/jacobianIK.hpp](solvers/jacobianIK.hpp) – inverse kinematics solver
- [test/unit_test_examples.cpp](test/unit_test_examples.cpp) – a small assertion-based sanity check for the math
- [ros2_plugin/custom_ik_moveit_planner/](ros2_plugin/custom_ik_moveit_planner/) – ROS 2 planner package skeleton
- [ros2_plugin/px150_moveit_config/](ros2_plugin/px150_moveit_config/) – MoveIt configuration assets for the PX150 robot model

## Build and run the C++ demo

```bash
mkdir -p build
g++ -std=c++17 -O2 -I. main.cpp -o build/ik_demo
./build/ik_demo
```

## Run the unit-test example

```bash
g++ -std=c++17 -O2 -I. test/unit_test_examples.cpp -o build/unit_test_examples
./build/unit_test_examples
```

## ROS 2 / MoveIt layout

This repo now also includes a lightweight ROS 2 package boundary for moving toward a MoveIt planner workflow.

### Recommended ROS 2 build pattern

From a ROS 2 workspace that contains this repository:

```bash
colcon build --packages-select custom_ik_moveit_planner px150_moveit_config
source install/setup.bash
```

### Launch the PX150 MoveIt config demo

```bash
ros2 launch px150_moveit_config demo.launch.py
```

### Launch the MoveIt move_group node directly

```bash
ros2 launch px150_moveit_config move_group.launch.py
```



