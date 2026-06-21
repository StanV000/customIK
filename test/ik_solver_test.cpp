#include <iostream>
#include <vector>
#include "../math/matrix.hpp"
#include "../math/transform.hpp"
#include "../kinematics/forward_kinematics/chain.hpp"
#include "../kinematics/forward_kinematics/joint.hpp"
#include "../solvers/jacobianIK.hpp"

void runIKSolverTest()
{
    std::cout << "Inverse Kinematics Solver Test " << std::endl;

    Joint j1(Transform::identity(), Axis::Z, 0.0);
    Joint j2(Transform(Matrix(1, 0, 0, 0, 1, 0, 0, 0, 1), Vector(1, 0, 0)), Axis::Z, 0.0);
    Joint endTip(Transform(Matrix(1, 0, 0, 0, 1, 0, 0, 0, 1), Vector(1, 0, 0)), Axis::Z, 0.0);

    Chain arm(Transform::identity(), std::vector<Joint>{j1, j2, endTip});
    arm.setAngle(0, 0.0);
    arm.setAngle(1, 0.0);
    arm.setAngle(2, 0.0);

    Transform target(Matrix(1, 0, 0, 0, 1, 0, 0, 0, 1), Vector(0, 2, 0));
    IKResult ikResult = solveIK(arm, target);

    std::cout << "IK success: " << (ikResult.success ? "yes" : "no")
              << "  iterations: " << ikResult.iterations << std::endl;

    Transform finalPose = arm.forwardKinematics();
    std::cout << "Final EE position: " << finalPose.getTranslation() << std::endl;

    constexpr double p = 3.14159265358979323846;
    std::cout << "Final joint angles (degrees): "
              << arm.joints[0].angle * 180.0 / p << ", "
              << arm.joints[1].angle * 180.0 / p << ", "
              << arm.joints[2].angle * 180.0 / p << std::endl;

    std::cout << std::endl;
}
