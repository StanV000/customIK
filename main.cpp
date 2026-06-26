#include <iostream>
#include "test/vector_transform_test.cpp"
#include "test/forward_kinematics_test.cpp"
#include "test/ik_solver_test.cpp"
#include "test/joint_limits_test.cpp"

int main()
{
    runIKSolverTest();
    runIKAccuracyTest();
    // runVectorTransformTest();
    // runForwardKinematicsTest();
    // runJointLimitTest();

    std::cout << "\n6-DOF Arm Test" << std::endl;

    Joint dof1(Transform::identity(), Axis::Z, 0.0);
    Joint dof2(Transform(Matrix(1, 0, 0, 0, 1, 0, 0, 0, 1), Vector(0, 0, 0.4)), Axis::Y, 0.0);
    Joint dof3(Transform(Matrix(1, 0, 0, 0, 1, 0, 0, 0, 1), Vector(0, 0, 0.5)), Axis::Y, 0.0);
    Joint dof4(Transform(Matrix(1, 0, 0, 0, 1, 0, 0, 0, 1), Vector(0, 0, 0.4)), Axis::Y, 0.0);
    Joint dof5(Transform(Matrix(1, 0, 0, 0, 1, 0, 0, 0, 1), Vector(0, 0, 0.35)), Axis::Z, 0.0);
    Joint dof6(Transform(Matrix(1, 0, 0, 0, 1, 0, 0, 0, 1), Vector(0, 0, 0.25)), Axis::Y, 0.0);
    Joint dofTip(Transform(Matrix(1, 0, 0, 0, 1, 0, 0, 0, 1), Vector(0, 0, 0.15)), Axis::Z, 0.0);

    Chain arm6dof(Transform::identity(),
                  std::vector<Joint>{dof1, dof2, dof3, dof4, dof5, dof6, dofTip});

    Transform fk6_1 = arm6dof.forwardKinematics();
    std::cout << "All angles 0: EE = " << fk6_1.getTranslation() << std::endl;

    arm6dof.setAngle(0, 1.5707963267948966);
    Transform fk6_2 = arm6dof.forwardKinematics();
    std::cout << "Joint1=90deg: EE = " << fk6_2.getTranslation() << std::endl;

    arm6dof.setAngle(0, 0.5);  // base ~30 deg, toward positive X/Y
    arm6dof.setAngle(1, 0.4);  // shoulder bend forward
    arm6dof.setAngle(2, -0.4); // elbow bend
    arm6dof.setAngle(3, 0.0);
    arm6dof.setAngle(4, 0.0);
    arm6dof.setAngle(5, 0.0);
    arm6dof.setAngle(6, 0.0);

    Transform target6d_1(Matrix(1, 0, 0, 0, 1, 0, 0, 0, 1), Vector(0.8, 0.5, 0.8));
    std::cout << "Solving IK for (0.8, 0.5, 0.8)..." << std::endl;
    IKResult res6d_1 = solveIK(arm6dof, target6d_1);
    std::cout << "IK success: " << (res6d_1.success ? "yes" : "no")
              << "  iterations: " << res6d_1.iterations << std::endl;
    std::cout << "Final EE: " << arm6dof.forwardKinematics().getTranslation() << std::endl;

    arm6dof.setAngle(0, 2.2);  // base pointing toward (-0.5, 0.7) region
    arm6dof.setAngle(1, 0.4);  // shoulder bend
    arm6dof.setAngle(2, -0.4); // elbow bend
    arm6dof.setAngle(3, 0.0);
    arm6dof.setAngle(4, 0.0);
    arm6dof.setAngle(5, 0.0);
    arm6dof.setAngle(6, 0.0);

    Transform target6d_2(Matrix(1, 0, 0, 0, 1, 0, 0, 0, 1), Vector(-0.5, 0.7, 0.6));
    std::cout << "Solving IK for (-0.5, 0.7, 0.6)..." << std::endl;
    IKResult res6d_2 = solveIK(arm6dof, target6d_2);
    std::cout << "IK success: " << (res6d_2.success ? "yes" : "no")
              << "  iterations: " << res6d_2.iterations << std::endl;
    std::cout << "Final EE: " << arm6dof.forwardKinematics().getTranslation() << std::endl;

    arm6dof.setAngle(0, -1.2); // base pointing toward (0.3, -0.8) region
    arm6dof.setAngle(1, 0.4);  // shoulder bend
    arm6dof.setAngle(2, -0.4); // elbow bend
    arm6dof.setAngle(3, 0.0);
    arm6dof.setAngle(4, 0.0);
    arm6dof.setAngle(5, 0.0);
    arm6dof.setAngle(6, 0.0);

    Transform target6d_3(Matrix(1, 0, 0, 0, 1, 0, 0, 0, 1), Vector(0.3, -0.8, 1.2));
    std::cout << "Solving IK for (0.3, -0.8, 1.2)..." << std::endl;
    IKResult res6d_3 = solveIK(arm6dof, target6d_3);
    std::cout << "IK success: " << (res6d_3.success ? "yes" : "no")
              << "  iterations: " << res6d_3.iterations << std::endl;
    std::cout << "Final EE: " << arm6dof.forwardKinematics().getTranslation() << std::endl;

    return 0;
}