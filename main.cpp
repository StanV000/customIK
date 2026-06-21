#include <iostream>
#include "math/vectors.hpp"
#include "math/matrix.hpp"
#include "math/transform.hpp"
#include "kinematics/forward_kinematics/chain.hpp"

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

    return 0;
}