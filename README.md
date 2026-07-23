
C++ project to learn about kinematics and inverse kinematics.

## What this project does

- Models a robotic arm as a chain of joints and offsets
- Computes forward kinematics by composing transforms
- Builds a Jacobian from the arm geometry
- Solves IK using a damped least-squares update


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
- [test/unit_test_examples.cpp](test/unit_test_examples.cpp) – a small assertion-based example to check the math

## Build and run

```bash
mkdir -p build
g++ -std=c++17 -O2 -I. main.cpp -o build/ik_demo
./build/ik_demo
```

# run unit-test

```bash
g++ -std=c++17 -O2 -I. test/unit_test_examples.cpp -o build/unit_test_examples
./build/unit_test_examples
```

