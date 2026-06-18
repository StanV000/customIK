
#include <cmath>
// gimbal lock, present three angles, RX, RY, RZ, roll, pitch, yaw.There are posisble configurations where you lose DoF, and can't rotate in certain directions. Quat don't have this.

// Interpolation, bending two oreitnations is hard, quat make it easier using slerp.

// storing matrix costs 9 doubles, quat only has 4, multiplying two matrices costs 27 multiplications, quat only has 16.
class Quaternion
{

    double w, x, y, z;

    Quaternion(double w, double x, double y, double z) : w(w), x(x), y(y), z(z) {}

    static Quaternion fromAxisAngle(double angle, double axisX, double axisY, double axisZ)
    {

        w = std::cos(angle / 2.0);
        x = axisX * std::sin(angle / 2.0);
        y = axisY * std::sin(angle / 2.0);
        z = axisZ * std::sin(angle / 2.0);

        return Quaternion(w, x, y, z);
    }

    Quaternion operator*(const Quaternion &q) const
    {
        return 0;
    }

    conjugate() const
    {
        return 0;
    }

    rotate(Vector v) const
    {
        return 0;
    }

    normalize() const
    {
        return 0;
    }

    toMatrix() const
    {
        return 0;
    }

    static Quaternion slerp(const Quaternion &q1, const Quaternion &q2, double t) const
    {
        return 0;
    }
}