
C++ project to learn about kinmeatics and IK.

## What this project does

- Models a robotic arm as a chain of joints and offsets
- Computes forward kinematics by composing transforms
- Builds a Jacobian for the end-effector motion
- Solves IK using a damped least-squares approach
- Includes small demo programs and test cases for solver behavior

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

In this project, the Jacobian is built from the arm’s geometry and the current joint axes.

### 3. Inverse kinematics
The solver uses a damped least-squares update:

$$\delta \theta = J^T (J J^T + \lambda I)^{-1} e$$

where $e$ is the position/orientation error between the target pose and the current pose.

The implementation also uses:
- adaptive damping to improve numerical stability
- a reachability check based on the approximate arm length
- quaternion-based orientation error for smoother handling of rotational error

## Project structure

- [main.cpp](main.cpp) – demo entry point that runs solver examples and tests
- [kinematics/forward_kinematics/](kinematics/forward_kinematics/) – joint and chain definitions
- [math/](math/) – vector, matrix, transform, quaternion, and Jacobian helpers
- [solvers/jacobianIK.hpp](solvers/jacobianIK.hpp) – inverse kinematics solver
- [test/](test/) – simple validation programs for transforms, FK, IK, and joint limits
- [ros2_plugin/](ros2_plugin/) – placeholder area for future ROS 2 integration

## Build and run

### Windows PowerShell

```powershell
mkdir build -Force
g++ -std=c++17 -O2 -I. main.cpp -o build/ik_demo
./build/ik_demo
```

### Linux / macOS

```bash
mkdir -p build
g++ -std=c++17 -O2 -I. main.cpp -o build/ik_demo
./build/ik_demo
```

