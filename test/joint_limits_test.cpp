#include <iostream>
#include <vector>
#include "../math/matrix.hpp"
#include "../math/transform.hpp"
#include "../kinematics/forward_kinematics/chain.hpp"
#include "../kinematics/forward_kinematics/joint.hpp"
#include "../solvers/jacobianIK.hpp"

void runJointLimitTest()
{
    std::cout << "Joint Limits and Limited Arm Test " << std::endl;

    Joint limitedJoint(Transform::identity(), Axis::Z, 0.0,
                       -0.7853981633974483, // -45 deg
                       0.7853981633974483); //  45 deg

    limitedJoint.setAngle(1.0);
    std::cout << "Tried 1.0 rad, got: " << limitedJoint.angle
              << " (expect 0.785..., i.e. 45 deg)" << std::endl;

    limitedJoint.setAngle(-2.0);
    std::cout << "Tried -2.0 rad, got: " << limitedJoint.angle
              << " (expect -0.785..., i.e. -45 deg)" << std::endl;

    limitedJoint.setAngle(0.3);
    std::cout << "Tried 0.3 rad, got: " << limitedJoint.angle
              << " (expect 0.3, unchanged)" << std::endl;

    Joint j1Limited(Transform::identity(), Axis::Z, 0.0,
                    -0.5235987755982988, // -30 deg
                    0.5235987755982988); //  30 deg

    Joint j2b(Transform(Matrix(1, 0, 0, 0, 1, 0, 0, 0, 1), Vector(1, 0, 0)), Axis::Z, 0.0);
    Joint endTipB(Transform(Matrix(1, 0, 0, 0, 1, 0, 0, 0, 1), Vector(1, 0, 0)), Axis::Z, 0.0);

    Chain limitedArm(Transform::identity(), std::vector<Joint>{j1Limited, j2b, endTipB});

    Transform target2(Matrix(1, 0, 0, 0, 1, 0, 0, 0, 1), Vector(0, 1.5, 0));
    IKResult limitedResult = solveIK(limitedArm, target2);

    std::cout << "Limited arm IK success: " << (limitedResult.success ? "yes" : "no")
              << "  iterations: " << limitedResult.iterations << std::endl;

    Transform limitedFinalPose = limitedArm.forwardKinematics();
    std::cout << "Limited arm final EE: " << limitedFinalPose.getTranslation() << std::endl;

    constexpr double p = 3.14159265358979323846;
    std::cout << "Limited arm final joint1 angle (deg): "
              << limitedArm.joints[0].angle * 180.0 / p << std::endl;

    std::cout << std::endl;
}
