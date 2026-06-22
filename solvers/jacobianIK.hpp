#pragma once
#include <vector>
#include <cmath>
#include "../math/vectors.hpp"
#include "../math/matrix.hpp"
#include "../math/transform.hpp"
#include "../kinematics/forward_kinematics/joint.hpp"
#include "../kinematics/forward_kinematics/chain.hpp"
#include "../math/jacobian.hpp"
#include "../math/matrixX.hpp"

struct IKResult
{
    bool success;
    int iterations;
};

inline Vector orientationError(const Matrix &targetR, const Matrix &currentR)
{
    Matrix Rerr = targetR * currentR.transpose();

    return Vector(
        0.5 * (Rerr.grid[2][1] - Rerr.grid[1][2]),
        0.5 * (Rerr.grid[0][2] - Rerr.grid[2][0]),
        0.5 * (Rerr.grid[1][0] - Rerr.grid[0][1]));
}

inline std::vector<double> dampedLeastSquares(const MatrixX &J, const std::vector<double> &error, double lambda)
{
    MatrixX Jt = J.transpose(); // Nx6
    MatrixX JJt = J * Jt;       // 6x6 (always square, regardless of N)
    JJt.addDamping(lambda);
    MatrixX JJt_inv = JJt.inverse(); // 6x6

    std::vector<double> temp = JJt_inv.multiply(error); // 6x1
    std::vector<double> dq = Jt.multiply(temp);         // Nx1

    return dq;
}

inline double computeMaxReach(const Chain &chain)
{
    double total = 0.0;
    for (const auto &joint : chain.joints)
    {
        Vector offset = joint.offset.getTranslation();
        total += offset.magnitude();
    }
    return total;
}

inline double computeManipulability(const MatrixX &J)
{
    MatrixX Jt = J.transpose();
    MatrixX JJt = J * Jt;
    double d = JJt.determinant();
    if (d < 0.0)
        d = 0.0; // numerical noise can give tiny negatives
    return std::sqrt(d);
}

// Nakamura-Hanafusa adaptive damping:
// returns a small lambda when well-conditioned,
// ramps up smoothly toward lambda_max as manipulability approaches zero
inline double adaptiveLambda(double w,
                             double w_threshold = 0.05,
                             double lambda_min = 0.001,
                             double lambda_max = 0.1)
{
    if (w >= w_threshold)
        return lambda_min;

    double t = 1.0 - w / w_threshold;
    return lambda_min + (lambda_max - lambda_min) * t * t;
}
inline IKResult solveIK(Chain &chain, const Transform &target, int maxIterations = 200, double tolerance = 1e-3, double stepSize = 0.5)
{
    double maxReach = computeMaxReach(chain);
    Vector baseToTarget = target.getTranslation() - chain.base.getTranslation();
    double targetDistance = baseToTarget.magnitude();

    if (targetDistance > maxReach)
    {
        std::cout << "IK target unreachable: distance " << targetDistance
                  << " exceeds max reach " << maxReach << std::endl;
        return {false, 0};
    }

    for (int iter = 0; iter < maxIterations; iter++)
    {
        Transform current = chain.forwardKinematics();

        Vector posError = target.getTranslation() - current.getTranslation();
        Vector rotError = orientationError(target.getRotation(), current.getRotation());

        std::vector<double> error = {
            posError.x, posError.y, posError.z,
            rotError.x, rotError.y, rotError.z};

        double errorMag = std::sqrt(
            posError.x * posError.x + posError.y * posError.y + posError.z * posError.z +
            rotError.x * rotError.x + rotError.y * rotError.y + rotError.z * rotError.z);

        if (errorMag < tolerance)
            return {true, iter};

        Jacobian J = computeJacobian(chain);
        MatrixX Jm = jacobianToMatrixX(J);
        double w = computeManipulability(Jm);
        double adaptedLambda = adaptiveLambda(w);

        if (iter % 10 == 0)
            std::cout << "  iter " << iter
                      << " error = " << errorMag
                      << " w = " << w
                      << " lambda = " << adaptedLambda
                      << std::endl;

        std::vector<double> dq = dampedLeastSquares(Jm, error, adaptedLambda);

        for (size_t i = 0; i < chain.joints.size(); i++)
        {
            double newAngle = chain.joints[i].angle + dq[i] * stepSize;
            chain.setAngle((int)i, newAngle);
        }
    }
    return {false, maxIterations};
}