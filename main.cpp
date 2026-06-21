#include <iostream>
#include "math/vectors.hpp"
#include "math/matrix.hpp"
#include "math/transform.hpp"
#include "math/jacobian.hpp"
#include "kinematics/forward_kinematics/chain.hpp"
#include "solvers/jacobianIK.hpp"
#include <numbers>

double p = std::numbers::pi;

int main()
{

    Vector a(1, 0, 0);
    Vector b(0, 1, 0);
    Vector sum = a + b;
    std::cout << "a + b = " << sum << std::endl;

    // build a 90 degree rotation around Z
    double angle = 1.5707963267948966; // pi/2, 90 degrees in radians
    Matrix rz90 = Matrix::Rz(angle);

    //  rotate (1,0,0) by 90 degrees around Z should be (0,1,0)
    Vector rotated = rz90 * a;
    std::cout << "Rz(90) * (1,0,0) = " << rotated << std::endl;

    // rotate 90 around Z, then shift by (2,0,0)
    Transform t(rz90, Vector(2, 0, 0));

    // (1,0,0)
    // (1,0,0) by 90 to (0,1,0) then (2,0,0) to (2,1,0)
    Vector result = t.transformPoint(a);
    std::cout << "Transform applied to (1,0,0) = " << result << std::endl;

    Joint j1(Transform::identity(), Axis::Z, 0.0);

    Joint j2(Transform(Matrix(1, 0, 0, 0, 1, 0, 0, 0, 1), Vector(1, 0, 0)), Axis::Z, 0.0);

    Joint endTip(Transform(Matrix(1, 0, 0, 0, 1, 0, 0, 0, 1), Vector(1, 0, 0)), Axis::Z, 0.0);

    Chain arm(Transform::identity(), std::vector<Joint>{j1, j2, endTip});

    Transform pose1 = arm.forwardKinematics();
    std::cout << "All angles 0: EE = " << pose1.getTranslation() << std::endl;

    arm.setAngle(0, 1.5707963267948966); // pi/2
    Transform pose2 = arm.forwardKinematics();
    std::cout << "Joint1 = 90deg: EE = " << pose2.getTranslation() << std::endl;

    arm.setAngle(0, 0.0);

    Jacobian J = computeJacobian(arm);

    std::cout << "Jacobian columns (Jv | Jw):" << std::endl;
    for (int i = 0; i < J.numJoints(); i++)
    {
        std::cout << "  joint " << i
                  << "  Jv = " << J.Jv[i]
                  << "  Jw = " << J.Jw[i]
                  << std::endl;
    }

    arm.setAngle(0, 0.0);
    arm.setAngle(1, 0.0);
    arm.setAngle(2, 0.0);

    Transform target(Matrix(1, 0, 0, 0, 1, 0, 0, 0, 1), Vector(0, 2, 0));

    IKResult ikResult = solveIK(arm, target);

    std::cout << "IK success: " << (ikResult.success ? "yes" : "no")
              << "  iterations: " << ikResult.iterations << std::endl;

    Transform finalPose = arm.forwardKinematics();
    std::cout << "Final EE position: " << finalPose.getTranslation() << std::endl;

    std::cout << "Final joint angles (degrees): "
              << arm.joints[0].angle * 180.0 / std::numbers::pi << ", "
              << arm.joints[1].angle * 180.0 / std::numbers::pi << ", "
              << arm.joints[2].angle * 180.0 / std::numbers::pi << std::endl;

    return 0;
}

// a + b = (1, 1, 0)
// Rz(90) * (1,0,0) = (6.12303e-17, 1, 0)
// Transform applied to (1,0,0) = (2, 1, 0)
// All angles 0: EE = (2, 0, 0)
// Joint1 = 90deg: EE = (1.22461e-16, 2, 0)
// Jacobian columns (Jv | Jw):
//   joint 0  Jv = (0, 2, 0)  Jw = (0, 0, 1)
//   joint 1  Jv = (0, 1, 0)  Jw = (0, 0, 1)
//   joint 2  Jv = (0, 0, 0)  Jw = (0, 0, 1)
//   iter 0 error = 2.82843
//   iter 10 error = 0.0106922
//   iter 20 error = 0.00261659
//   iter 30 error = 0.00144966
//   iter 40 error = 0.000993151
// IK success: yes  iterations: 40
// Final EE position: (-6.03638e-09, 1.99901, 0)
// Final joint angles (degrees): 88.1943, 3.61132, -91.80