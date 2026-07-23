#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

#include "../math/matrix.hpp"
#include "../math/transform.hpp"
#include "../kinematics/forward_kinematics/chain.hpp"
#include "../kinematics/forward_kinematics/joint.hpp"
#include "../solvers/jacobianIK.hpp"

static void expectClose(double actual, double expected, double tol = 1e-6)
{
    assert(std::abs(actual - expected) < tol && "value out of tolerance");
}

static void expectVectorClose(const Vector &actual, const Vector &expected, double tol = 1e-6)
{
    expectClose(actual.x, expected.x, tol);
    expectClose(actual.y, expected.y, tol);
    expectClose(actual.z, expected.z, tol);
}

int main()
{
    Matrix rz90 = Matrix::Rz(0.5 * pi);
    Vector rotated = rz90 * Vector(1.0, 0.0, 0.0);
    expectVectorClose(rotated, Vector(0.0, 1.0, 0.0), 1e-9);

    Transform t(rz90, Vector(2.0, 0.0, 0.0));
    Vector point = t.transformPoint(Vector(1.0, 0.0, 0.0));
    expectVectorClose(point, Vector(2.0, 1.0, 0.0), 1e-9);

    Joint j1(Transform(Matrix::identity(), Vector(1.0, 0.0, 0.0)), Axis::Z, 0.0);
    Joint j2(Transform(Matrix::identity(), Vector(1.0, 0.0, 0.0)), Axis::Z, 0.0);
    Chain arm(Transform::identity(), std::vector<Joint>{j1, j2});
    Transform pose = arm.forwardKinematics();
    expectVectorClose(pose.getTranslation(), Vector(2.0, 0.0, 0.0), 1e-9);

    Joint limited(Transform::identity(), Axis::Z, 0.0, -0.5, 0.5);
    limited.setAngle(1.0);
    expectClose(limited.angle, 0.5, 1e-9);

    Joint ik1(Transform::identity(), Axis::Z, 0.0);
    Joint ik2(Transform(Matrix::identity(), Vector(1.0, 0.0, 0.0)), Axis::Z, 0.0);
    Joint ikTip(Transform(Matrix::identity(), Vector(1.0, 0.0, 0.0)), Axis::Z, 0.0);
    Chain ikArm(Transform::identity(), std::vector<Joint>{ik1, ik2, ikTip});


    Transform target(Matrix::identity(), Vector(2.0, 0.0, 0.0));
    IKResult result = solveIK(ikArm, target, 200, 1e-4, 0.5);
    assert(result.success && "expected IK solve to succeed");

    Transform finalPose = ikArm.forwardKinematics();
    double positionError = (target.getTranslation() - finalPose.getTranslation()).magnitude();
    assert(positionError < 1e-3 && "expected target position error to be small");

    std::cout << "All unit-test examples passed." << std::endl;
    return 0;
}
