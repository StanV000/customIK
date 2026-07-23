#include <iostream>
#include <iomanip>
#include <string>
#include <vector>

#include "math/vectors.hpp"
#include "math/matrix.hpp"
#include "math/transform.hpp"
#include "math/jacobian.hpp"
#include "math/matrixX.hpp"
#include "kinematics/forward_kinematics/joint.hpp"
#include "kinematics/forward_kinematics/chain.hpp"
#include "solvers/jacobianIK.hpp"

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
              << std::left << std::setw(18) << label << ": ("
              << std::setw(8) << v.x << ", "
              << std::setw(8) << v.y << ", "
              << std::setw(8) << v.z << ")\n";
}

void printTransform(const std::string &label, const Transform &t)
{
    std::cout << "\n" << label << "\n";
    printVector("Translation", t.getTranslation());
    const Matrix &R = t.getRotation();
    std::cout << std::fixed << std::setprecision(4)
              << "  [ " << std::setw(8) << R.grid[0][0] << "  " << std::setw(8) << R.grid[0][1] << "  " << std::setw(8) << R.grid[0][2] << " ]\n"
              << "  [ " << std::setw(8) << R.grid[1][0] << "  " << std::setw(8) << R.grid[1][1] << "  " << std::setw(8) << R.grid[1][2] << " ]\n"
              << "  [ " << std::setw(8) << R.grid[2][0] << "  " << std::setw(8) << R.grid[2][1] << "  " << std::setw(8) << R.grid[2][2] << " ]\n";
}

void printJacobianStep(const std::string &label, const Vector &z_i, const Vector &p_i, const Vector &p_e)
{
    Vector jv = z_i.cross(p_e - p_i);
    std::cout << label << "\n";
    printVector("  z_i", z_i);
    printVector("  p_i", p_i);
    printVector("  p_e", p_e);
    printVector("  Jv_i = z_i x (p_e - p_i)", jv);
    printVector("  Jw_i = z_i", z_i);
}

} // namespace

int main()
{
    printSection("Learning walkthrough: forward kinematics + Jacobian");
    std::cout << "Formula 1: T_local = T_offset * R(axis, theta)\n";
    std::cout << "Formula 2: T_ee = T_base * T_1 * T_2 * ... * T_n\n\n";

    Joint j1(Transform(Matrix::identity(), Vector(1.0, 0.0, 0.0)), Axis::Z, 0.0);
    Joint j2(Transform(Matrix::identity(), Vector(1.0, 0.0, 0.0)), Axis::Z, 0.0);
    Chain arm(Transform::identity(), std::vector<Joint>{j1, j2});

    const double theta1 = 0.5 * pi;
    const double theta2 = -0.5 * pi;
    arm.setAngle(0, theta1);
    arm.setAngle(1, theta2);

    Transform T0 = arm.base;
    Transform T1 = T0 * arm.joints[0].localTransform();
    Transform T2 = T1 * arm.joints[1].localTransform();
    Transform T_fk = arm.forwardKinematics();

    printTransform("T0 = base", T0);
    printTransform("T1 = T0 * joint1.localTransform()", T1);
    printTransform("T2 = T1 * joint2.localTransform()", T2);
    printTransform("T_fk = arm.forwardKinematics()", T_fk);

    std::cout << "\nNotice the repeated chain rule: each joint contributes another transform.\n";

    std::vector<Transform> transforms;
    Transform accum = arm.base;
    for (const auto &joint : arm.joints)
    {
        accum = accum * joint.localTransform();
        transforms.push_back(accum);
    }

    std::cout << "\nJacobian formula: Jv_i = z_i x (p_e - p_i), Jw_i = z_i\n";
    Vector p_e = transforms.back().getTranslation();
    for (size_t i = 0; i < arm.joints.size(); ++i)
    {
        Vector p_i = transforms[i].getTranslation();
        Vector localAxis = Vector(0.0, 0.0, 1.0);
        Vector z_i = transforms[i].getRotation() * localAxis;
        printJacobianStep("\nJoint " + std::to_string(i) + " contribution", z_i, p_i, p_e);
    }

    Jacobian J = computeJacobian(arm);
    MatrixX Jm = jacobianToMatrixX(J);
    std::cout << "\nJacobian matrix (6 x " << Jm.cols << ") after jacobianToMatrixX():\n";
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

    printSection("IK solve on the same arm");
    Transform target(Matrix::identity(), Vector(1.2, 0.6, 0.0));
    printVector("Target position", target.getTranslation());
    IKResult result = solveIK(arm, target, 200, 1e-4, 0.5);
    std::cout << "IK success: " << (result.success ? "yes" : "no")
              << " | iterations=" << result.iterations << "\n";

    return 0;
}
