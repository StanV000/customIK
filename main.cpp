#include <iostream>
#include "test/vector_transform_test.cpp"
#include "test/forward_kinematics_test.cpp"
#include "test/ik_solver_test.cpp"
#include "test/joint_limits_test.cpp"

int main()
{
    runVectorTransformTest();
    runForwardKinematicsTest();
    runIKSolverTest();
    runJointLimitTest();
    return 0;
}