#pragma once
#include <iostream>
#include <iomanip>
#include <vector>
#include <cmath>
#include "../math/vectors.hpp"
#include "../math/matrix.hpp"
#include "../math/transform.hpp"
#include "../math/quaternion.hpp"
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
    Quaternion qt = Quaternion::fromRotationMatrix(targetR);
    Quaternion qc = Quaternion::fromRotationMatrix(currentR);
    Quaternion qerr = qt * qc.conjugate();
    if (qerr.w < 0.0)
        qerr = Quaternion(-qerr.w, -qerr.x, -qerr.y, -qerr.z);
    return Vector(qerr.x, qerr.y, qerr.z) * 2.0;
}

inline std::vector<double> dampedLeastSquares(const MatrixX &J, const std::vector<double> &error, double lambda)
{
    MatrixX Jt = J.transpose(); // Nx6
    MatrixX JJt = J * Jt;       // 6x6 (always square, regardless of N)
    JJt.addDamping(lambda);

    std::vector<double> temp;
    try
    {
        temp = JJt.solve(error); // 6x1
    }
    catch (const std::runtime_error &)
    {
        // fallback: increase damping if singular
        JJt.addDamping(lambda * 10.0);
        temp = JJt.solve(error);
    }

    std::vector<double> dq = Jt.multiply(temp); // Nx1

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
inline void printMatrixXDebug(const MatrixX &m, const std::string &name)
{
    std::cout << "\n" << name << " (" << m.rows << "x" << m.cols << ")\n";
    for (int r = 0; r < m.rows; ++r)
    {
        std::cout << "  [";
        for (int c = 0; c < m.cols; ++c)
        {
            std::cout << std::fixed << std::setprecision(4) << std::setw(10) << m(r, c);
            if (c + 1 < m.cols)
                std::cout << " ";
        }
        std::cout << "]\n";
    }
}

inline IKResult solveIK(Chain &chain, const Transform &target, int maxIterations = 200, double tolerance = 1e-3, double stepSize = 0.5)
{
    double maxReach = computeMaxReach(chain);
    Vector baseToTarget = target.getTranslation() - chain.base.getTranslation();
    double targetDistance = baseToTarget.magnitude();

    bool unreachable = false;
    if (targetDistance > maxReach)
    {
        unreachable = true;
        std::cout << "IK target unreachable: distance " << targetDistance
                  << " exceeds max reach " << maxReach
                  << ". Solver will still approximate closest reachable pose." << std::endl;
    }

    for (int iter = 0; iter < maxIterations; iter++)
    {
        Transform current = chain.forwardKinematics();

        Vector posError = target.getTranslation() - current.getTranslation();
        Vector rotError = orientationError(target.getRotation(), current.getRotation());

        double positionWeight = 1.0;
        double orientationWeight = 0.5;

        std::vector<double> error = {
            posError.x * positionWeight,
            posError.y * positionWeight,
            posError.z * positionWeight,
            rotError.x * orientationWeight,
            rotError.y * orientationWeight,
            rotError.z * orientationWeight};

        double posErrorMag = posError.magnitude();
        double rotErrorMag = rotError.magnitude();
        double errorMag = std::sqrt(
            posError.x * posError.x + posError.y * posError.y + posError.z * posError.z +
            rotError.x * rotError.x + rotError.y * rotError.y + rotError.z * rotError.z);

        if (posErrorMag < tolerance && rotErrorMag < tolerance * 0.5)
            return {true, iter};

        Jacobian J = computeJacobian(chain);
        MatrixX Jm = jacobianToMatrixX(J);
        double w = computeManipulability(Jm);
        double adaptedLambda = adaptiveLambda(w);

        if (iter == 0)
        {
            std::cout << "\n[IK] Starting solve" << std::endl;
            printMatrixXDebug(Jm, "Jacobian");
        }

        if (iter % 10 == 0 || iter + 1 == maxIterations)
            std::cout << "[IK] iter " << std::setw(3) << iter
                      << " | posErr=" << std::fixed << std::setprecision(4) << std::setw(9) << posErrorMag
                      << " | rotErr=" << std::setw(9) << rotErrorMag
                      << " | manipulability=" << std::setw(9) << w
                      << " | lambda=" << std::setw(9) << adaptedLambda
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