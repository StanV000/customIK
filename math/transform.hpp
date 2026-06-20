#pragma once
#include "vectors.hpp"
#include "matrix.hpp"

class Transform
{
public:
    Matrix R;
    Vector t;

    Transform(const Matrix &R, const Vector &t) : R(R), t(t) {}

    static Transform identity()
    {
        return Transform(Matrix(1, 0, 0, 0, 1, 0, 0, 0, 1), Vector(0, 0, 0));
    }

    Transform operator*(const Transform &other) const
    {
        Matrix newR = R * other.R;
        Vector newT = (R * other.t) + t;
        return Transform(newR, newT);
    }

    Vector transformPoint(const Vector &p) const
    {
        return (R * p) + t;
    }

    Vector transformDirection(const Vector &d) const
    {
        return R * d;
    }

    Transform inverse() const
    {
        Matrix Rt = R.transpose();
        Vector newT = Rt * t * -1.0;
        return Transform(Rt, newT);
    }

    Matrix getRotation() const
    {
        return R;
    }

    Vector getTranslation() const
    {
        return t;
    }
};