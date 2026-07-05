#include <iostream>
#include <iomanip>
#include <string>
#include "test/vector_transform_test.cpp"
#include "test/forward_kinematics_test.cpp"
#include "test/ik_solver_test.cpp"
#include "test/joint_limits_test.cpp"

namespace {

void printSection(const std::string &title)
{
    std::cout << "\n" << std::string(70, '=') << "\n";
    std::cout << "  " << title << "\n";
    std::cout << std::string(70, '=') << "\n";
}

void printVector(const std::string &label, const Vector &v)
{
    std::cout << std::fixed << std::setprecision(4)
              << std::left << std::setw(16) << label << ": ("
              << std::setw(8) << v.x << ", "
              << std::setw(8) << v.y << ", "
              << std::setw(8) << v.z << ")\n";
}

void printPoseSummary(const std::string &label, const Transform &pose)
{
    printSection(label);
    printVector("Translation", pose.getTranslation());
    std::cout << std::left << std::setw(16) << "Rotation" << ": matrix form\n";
    const Matrix &R = pose.getRotation();
    std::cout << std::fixed << std::setprecision(4)
              << "  [ " << std::setw(8) << R.grid[0][0] << "  " << std::setw(8) << R.grid[0][1] << "  " << std::setw(8) << R.grid[0][2] << " ]\n"
              << "  [ " << std::setw(8) << R.grid[1][0] << "  " << std::setw(8) << R.grid[1][1] << "  " << std::setw(8) << R.grid[1][2] << " ]\n"
              << "  [ " << std::setw(8) << R.grid[2][0] << "  " << std::setw(8) << R.grid[2][1] << "  " << std::setw(8) << R.grid[2][2] << " ]\n";
}

void printJointAngles(const Chain &chain)
{
    std::cout << std::left << std::setw(14) << "Joint angles" << ": ";
    for (size_t i = 0; i < chain.joints.size(); ++i)
    {
        std::cout << std::fixed << std::setprecision(3)
                  << "J" << i << "=" << chain.joints[i].angle << "rad";
        if (i + 1 < chain.joints.size())
            std::cout << ", ";
    }
    std::cout << "\n";
}

void printJacobian(const Chain &chain)
{
    Jacobian J = computeJacobian(chain);
    MatrixX Jm = jacobianToMatrixX(J);

    std::cout << "\nJacobian (6x" << Jm.cols << ")\n";
    for (int r = 0; r < Jm.rows; ++r)
    {
        std::cout << "  [";
        for (int c = 0; c < Jm.cols; ++c)
        {
            std::cout << std::fixed << std::setprecision(4) << std::setw(10) << Jm(r, c);
            if (c + 1 < Jm.cols)
                std::cout << " ";
        }
        std::cout << "]\n";
    }
}

void printIKSummary(const std::string &label, const IKResult &result)
{
    std::cout << std::left << std::setw(18) << label << ": "
              << (result.success ? "success" : "failed")
              << " | iterations=" << result.iterations << "\n";
}

} // namespace

int main()
{
    runIKSolverTest();
    runIKAccuracyTest();
    // runVectorTransformTest();
    // runForwardKinematicsTest();
    // runJointLimitTest();

    printSection("6-DOF Arm Demonstration");

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
    printPoseSummary("Initial pose (all angles = 0)", fk6_1);
    printJointAngles(arm6dof);
    printJacobian(arm6dof);

    arm6dof.setAngle(0, 1.5707963267948966);
    Transform fk6_2 = arm6dof.forwardKinematics();
    printPoseSummary("Pose after rotating joint 1 by 90 degrees", fk6_2);
    printJointAngles(arm6dof);

    arm6dof.setAngle(0, 0.5);  // base ~30 deg, toward positive X/Y
    arm6dof.setAngle(1, 0.4);  // shoulder bend forward
    arm6dof.setAngle(2, -0.4); // elbow bend
    arm6dof.setAngle(3, 0.0);
    arm6dof.setAngle(4, 0.0);
    arm6dof.setAngle(5, 0.0);
    arm6dof.setAngle(6, 0.0);

    Transform target6d_1(Matrix(1, 0, 0, 0, 1, 0, 0, 0, 1), Vector(0.8, 0.5, 0.8));
    printSection("IK Solve #1");
    printVector("Target position", target6d_1.getTranslation());
    std::cout << "Starting joint state:\n";
    printJointAngles(arm6dof);
    printJacobian(arm6dof);
    IKResult res6d_1 = solveIK(arm6dof, target6d_1);
    printIKSummary("Result", res6d_1);
    printPoseSummary("Final end-effector pose", arm6dof.forwardKinematics());
    printJointAngles(arm6dof);

    arm6dof.setAngle(0, 2.2);  // base pointing toward (-0.5, 0.7) region
    arm6dof.setAngle(1, 0.4);  // shoulder bend
    arm6dof.setAngle(2, -0.4); // elbow bend
    arm6dof.setAngle(3, 0.0);
    arm6dof.setAngle(4, 0.0);
    arm6dof.setAngle(5, 0.0);
    arm6dof.setAngle(6, 0.0);

    Transform target6d_2(Matrix(1, 0, 0, 0, 1, 0, 0, 0, 1), Vector(-0.5, 0.7, 0.6));
    printSection("IK Solve #2");
    printVector("Target position", target6d_2.getTranslation());
    std::cout << "Starting joint state:\n";
    printJointAngles(arm6dof);
    printJacobian(arm6dof);
    IKResult res6d_2 = solveIK(arm6dof, target6d_2);
    printIKSummary("Result", res6d_2);
    printPoseSummary("Final end-effector pose", arm6dof.forwardKinematics());
    printJointAngles(arm6dof);

    arm6dof.setAngle(0, -1.2); // base pointing toward (0.3, -0.8) region
    arm6dof.setAngle(1, 0.4);  // shoulder bend
    arm6dof.setAngle(2, -0.4); // elbow bend
    arm6dof.setAngle(3, 0.0);
    arm6dof.setAngle(4, 0.0);
    arm6dof.setAngle(5, 0.0);
    arm6dof.setAngle(6, 0.0);

    Transform target6d_3(Matrix(1, 0, 0, 0, 1, 0, 0, 0, 1), Vector(0.3, -0.8, 1.2));
    printSection("IK Solve #3");
    printVector("Target position", target6d_3.getTranslation());
    std::cout << "Starting joint state:\n";
    printJointAngles(arm6dof);
    printJacobian(arm6dof);
    IKResult res6d_3 = solveIK(arm6dof, target6d_3);
    printIKSummary("Result", res6d_3);
    printPoseSummary("Final end-effector pose", arm6dof.forwardKinematics());
    printJointAngles(arm6dof);

    return 0;
}