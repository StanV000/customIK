#include <iostream>
#include <vector>
#include "../math/matrix.hpp"
#include "../math/transform.hpp"
#include "../kinematics/forward_kinematics/chain.hpp"
#include "../kinematics/forward_kinematics/joint.hpp"

void runForwardKinematicsTest()
{
    std::cout << "Forward Kinematics Test " << std::endl;

    Joint j1(Transform::identity(), Axis::Z, 0.0);
    Joint j2(Transform(Matrix(1, 0, 0, 0, 1, 0, 0, 0, 1), Vector(1, 0, 0)), Axis::Z, 0.0);
    Joint endTip(Transform(Matrix(1, 0, 0, 0, 1, 0, 0, 0, 1), Vector(1, 0, 0)), Axis::Z, 0.0);

    Chain arm(Transform::identity(), std::vector<Joint>{j1, j2, endTip});

    Transform pose1 = arm.forwardKinematics();
    std::cout << "All angles 0: EE = " << pose1.getTranslation() << std::endl;

    arm.setAngle(0, 1.5707963267948966); // pi/2
    Transform pose2 = arm.forwardKinematics();
    std::cout << "Joint1 = 90deg: EE = " << pose2.getTranslation() << std::endl;

    std::cout << std::endl;
}
