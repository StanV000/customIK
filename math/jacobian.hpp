#pragma once
#include <vector>
#include "vectors.hpp"
#include "matrix.hpp"
#include "transform.hpp"
#include "../kinematics/forward_kinematics/joint.hpp"
#include "../kinematics/forward_kinematics/chain.hpp"
#include "matrixX.hpp"

inline Vector axisToVector(Axis axis)
{
    if (axis == Axis::X)
        return Vector(1, 0, 0);
    if (axis == Axis::Y)
        return Vector(0, 1, 0);
    return Vector(0, 0, 1);
}

class Jacobian
{
public:
    std::vector<Vector> Jv; // linear part, one vector per joint
    std::vector<Vector> Jw; // angular part, one vector per joint

    int numJoints() const
    {
        return (int)Jv.size();
    }
};

inline Jacobian computeJacobian(const Chain &chain)
{
    Jacobian J;

    std::vector<Transform> transforms;
    Transform T = chain.base;
    for (const auto &joint : chain.joints)
    {
        T = T * joint.localTransform();
        transforms.push_back(T);
    }

    Vector p_e = transforms.back().getTranslation();

    for (size_t i = 0; i < chain.joints.size(); i++)
    {
        Vector p_i = transforms[i].getTranslation();

        Vector localAxis = axisToVector(chain.joints[i].axis);
        Vector z_i = transforms[i].getRotation() * localAxis;

        Vector Jv_i = z_i.cross(p_e - p_i);
        Vector Jw_i = z_i;

        J.Jv.push_back(Jv_i);
        J.Jw.push_back(Jw_i);
    }

    return J;
}

inline MatrixX jacobianToMatrixX(const Jacobian &J)
{
    int n = J.numJoints();
    MatrixX m(6, n);

    for (int i = 0; i < n; i++)
    {
        m(0, i) = J.Jv[i].x;
        m(1, i) = J.Jv[i].y;
        m(2, i) = J.Jv[i].z;
        m(3, i) = J.Jw[i].x;
        m(4, i) = J.Jw[i].y;
        m(5, i) = J.Jw[i].z;
    }

    return m;
}
