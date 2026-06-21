#pragma once
#include "../../math/vectors.hpp"
#include "../../math/matrix.hpp"
#include "../../math/transform.hpp"

constexpr double pi = 3.14159265358979323846;

enum class Axis
{
    X,
    Y,
    Z
};

class Joint
{
public:
    Transform offset; // offset from parent joint
    Axis axis;        // axis of rotation
    double angle;     // current angle of rotation
    double minAngle;  // rad
    double maxAngle;  // rad

    Joint(const Transform &offset, Axis axis, double angle = 0.0, double minAngle = -pi, double maxAngle = pi)
        : offset(offset), axis(axis), angle(angle), minAngle(minAngle), maxAngle(maxAngle) {}

    Transform rotationTransform() const
    {
        Matrix R(1, 0, 0, 0, 1, 0, 0, 0, 1);

        if (axis == Axis::X)
        {
            R = Matrix::Rx(angle);
        }
        else if (axis == Axis::Y)
        {
            R = Matrix::Ry(angle);
        }
        else
        {
            R = Matrix::Rz(angle);
        }

        return Transform(R, Vector(0, 0, 0));
    }

    Transform localTransform() const
    {
        return offset * rotationTransform();
    }

    void setAngle(double newAngle)
    {
        angle = clamp(newAngle, minAngle, maxAngle);
    }

private:
    static double clamp(double value, double lo, double hi)
    {
        if (value < lo)
            return lo;
        if (value > hi)
            return hi;
        return value;
    };
};
