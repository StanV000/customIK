#include <iostream>
#include <vector>
#include <random>
#include <iomanip>
#include "../math/matrix.hpp"
#include "../math/transform.hpp"
#include "../kinematics/forward_kinematics/chain.hpp"
#include "../kinematics/forward_kinematics/joint.hpp"
#include "../solvers/jacobianIK.hpp"

static double computePositionError(const Transform &target, const Transform &current)
{
    return (target.getTranslation() - current.getTranslation()).magnitude();
}

static double computeOrientationErrorMag(const Transform &target, const Transform &current)
{
    return orientationError(target.getRotation(), current.getRotation()).magnitude();
}

static std::vector<double> getJointAngles(const Chain &arm)
{
    std::vector<double> angles;
    angles.reserve(arm.joints.size());
    for (const auto &joint : arm.joints)
        angles.push_back(joint.angle);
    return angles;
}

void runIKAccuracyTest(int samples = 20)
{
    std::cout << "Inverse Kinematics Accuracy Benchmark" << std::endl;

    Joint j1(Transform::identity(), Axis::Z, 0.0);
    Joint j2(Transform(Matrix(1, 0, 0, 0, 1, 0, 0, 0, 1), Vector(1, 0, 0)), Axis::Z, 0.0);
    Joint endTip(Transform(Matrix(1, 0, 0, 0, 1, 0, 0, 0, 1), Vector(1, 0, 0)), Axis::Z, 0.0);

    Chain arm(Transform::identity(), std::vector<Joint>{j1, j2, endTip});
    double maxReach = computeMaxReach(arm);

    std::mt19937 rng(123456);
    std::uniform_real_distribution<double> radiusDist(0.2 * maxReach, 0.95 * maxReach);
    std::uniform_real_distribution<double> azimuthDist(0.0, 2.0 * pi);
    std::uniform_real_distribution<double> initAngleDist(-1.0, 1.0);

    int successCount = 0;
    double totalPosError = 0.0;
    double totalOriError = 0.0;
    double maxPosError = 0.0;
    double maxOriError = 0.0;

    for (int sample = 0; sample < samples; ++sample)
    {
        double radius = radiusDist(rng);
        double az = azimuthDist(rng);
        Vector targetPos(radius * std::cos(az), radius * std::sin(az), 0.0);

        Transform target(Transform::identity().getRotation(), targetPos);

        for (size_t i = 0; i < arm.joints.size(); ++i)
            arm.setAngle((int)i, initAngleDist(rng));

        auto beforeAngles = getJointAngles(arm);

        IKResult result = solveIK(arm, target, 200, 1e-4, 0.5);
        Transform finalPose = arm.forwardKinematics();

        double posError = computePositionError(target, finalPose);
        double oriError = computeOrientationErrorMag(target, finalPose);

        totalPosError += posError;
        totalOriError += oriError;
        maxPosError = std::max(maxPosError, posError);
        maxOriError = std::max(maxOriError, oriError);
        if (result.success)
            successCount++;

        std::cout << "target " << sample << " = " << targetPos
                  << "  success=" << (result.success ? "yes" : "no")
                  << "  iters=" << result.iterations
                  << "  posError=" << std::fixed << std::setprecision(5) << posError
                  << "  oriError=" << std::fixed << std::setprecision(5) << oriError
                  << "  jointDelta=[";

        auto afterAngles = getJointAngles(arm);
        for (size_t i = 0; i < afterAngles.size(); ++i)
        {
            double delta = std::abs(afterAngles[i] - beforeAngles[i]);
            std::cout << std::fixed << std::setprecision(3) << delta;
            if (i + 1 < afterAngles.size())
                std::cout << ", ";
        }
        std::cout << "]" << std::endl;
    }

    std::cout << "Summary: " << successCount << " / " << samples << " successes"
              << "  avgPos=" << std::fixed << std::setprecision(5) << (totalPosError / samples)
              << "  maxPos=" << maxPosError
              << "  avgOri=" << (totalOriError / samples)
              << "  maxOri=" << maxOriError << std::endl
              << std::endl;
}

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
