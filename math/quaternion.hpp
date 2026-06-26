#pragma once
#include <cmath>
#include "vectors.hpp"
#include "matrix.hpp"
// gimbal lock, present three angles, RX, RY, RZ, roll, pitch, yaw.There are posisble configurations where you lose DoF, and can't rotate in certain directions. Quat don't have this.

// Interpolation, bending two oreitnations is hard, quat make it easier using slerp.

// storing matrix costs 9 doubles, quat only has 4, multiplying two matrices costs 27 multiplications, quat only has 16.
class Quaternion
{
public:
    double w, x, y, z;

    Quaternion() : w(1.0), x(0.0), y(0.0), z(0.0) {}
    Quaternion(double w, double x, double y, double z) : w(w), x(x), y(y), z(z) {}

    static Quaternion fromAxisAngle(double angle, double axisX, double axisY, double axisZ)
    {
        double half = angle * 0.5;
        double s = std::sin(half);
        double w = std::cos(half);
        double x = axisX * s;
        double y = axisY * s;
        double z = axisZ * s;
        return Quaternion(w, x, y, z).normalize();
    }

    Quaternion operator*(const Quaternion &q) const
    {
        double w_res = w * q.w - x * q.x - y * q.y - z * q.z;
        double x_res = w * q.x + x * q.w + y * q.z - z * q.y;
        double y_res = w * q.y - x * q.z + y * q.w + z * q.x;
        double z_res = w * q.z + x * q.y - y * q.x + z * q.w;
        return Quaternion(w_res, x_res, y_res, z_res);
    }

    Quaternion conjugate() const
    {
        return Quaternion(w, -x, -y, -z);
    }

    // TOOD see the optimized verson by using  vector expanasion formula, can break this into 15 multiplications and additions.
    // TODO
    Vector rotate(const Vector &v) const
    {
        Quaternion qv(0.0, v.x, v.y, v.z);
        Quaternion q_conj = conjugate();
        Quaternion q_rotated = (*this) * qv * q_conj;
        return Vector(q_rotated.x, q_rotated.y, q_rotated.z);
    }

    Quaternion normalize() const
    {
        double norm = std::sqrt(w * w + x * x + y * y + z * z);
        if (norm == 0.0)
            return Quaternion(1.0, 0.0, 0.0, 0.0);
        return Quaternion(w / norm, x / norm, y / norm, z / norm);
    }

    double dot(const Quaternion &q) const
    {
        return w * q.w + x * q.x + y * q.y + z * q.z;
    }

    static Quaternion fromRotationMatrix(const Matrix &m)
    {
        double trace = m.grid[0][0] + m.grid[1][1] + m.grid[2][2];
        if (trace > 0.0)
        {
            double s = std::sqrt(trace + 1.0) * 2.0;
            double w = 0.25 * s;
            double x = (m.grid[2][1] - m.grid[1][2]) / s;
            double y = (m.grid[0][2] - m.grid[2][0]) / s;
            double z = (m.grid[1][0] - m.grid[0][1]) / s;
            return Quaternion(w, x, y, z).normalize();
        }
        if (m.grid[0][0] > m.grid[1][1] && m.grid[0][0] > m.grid[2][2])
        {
            double s = std::sqrt(1.0 + m.grid[0][0] - m.grid[1][1] - m.grid[2][2]) * 2.0;
            double w = (m.grid[2][1] - m.grid[1][2]) / s;
            double x = 0.25 * s;
            double y = (m.grid[0][1] + m.grid[1][0]) / s;
            double z = (m.grid[0][2] + m.grid[2][0]) / s;
            return Quaternion(w, x, y, z).normalize();
        }
        if (m.grid[1][1] > m.grid[2][2])
        {
            double s = std::sqrt(1.0 + m.grid[1][1] - m.grid[0][0] - m.grid[2][2]) * 2.0;
            double w = (m.grid[0][2] - m.grid[2][0]) / s;
            double x = (m.grid[0][1] + m.grid[1][0]) / s;
            double y = 0.25 * s;
            double z = (m.grid[1][2] + m.grid[2][1]) / s;
            return Quaternion(w, x, y, z).normalize();
        }
        double s = std::sqrt(1.0 + m.grid[2][2] - m.grid[0][0] - m.grid[1][1]) * 2.0;
        double w = (m.grid[1][0] - m.grid[0][1]) / s;
        double x = (m.grid[0][2] + m.grid[2][0]) / s;
        double y = (m.grid[1][2] + m.grid[2][1]) / s;
        double z = 0.25 * s;
        return Quaternion(w, x, y, z).normalize();
    }

    Matrix toMatrix() const
    {
        double xx = x * x;
        double yy = y * y;
        double zz = z * z;
        double xy = x * y;
        double xz = x * z;
        double yz = y * z;
        double wx = w * x;
        double wy = w * y;
        double wz = w * z;

        return Matrix(
            1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy),
            2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx),
            2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy));
    }

    static Quaternion slerp(const Quaternion &q1, const Quaternion &q2, double t)
    {
        double dot = q1.w * q2.w + q1.x * q2.x + q1.y * q2.y + q1.z * q2.z;

        Quaternion b = q2;

        if (dot < 0.0)
        {
            b = Quaternion(-b.w, -b.x, -b.y, -b.z);
            dot = -dot;
        }

        const double DOT_THRESHOLD = 0.9995;
        if (dot > DOT_THRESHOLD)
        {
            return Quaternion(
                q1.w + t * (b.w - q1.w),
                q1.x + t * (b.x - q1.x),
                q1.y + t * (b.y - q1.y),
                q1.z + t * (b.z - q1.z))
                .normalize();
        }

        double theta_0 = std::acos(dot);
        double theta = theta_0 * t;

        double sin_theta = std::sin(theta);
        double sin_theta_0 = std::sin(theta_0);
        double s1 = std::sin(theta_0 - theta) / sin_theta_0;
        double s2 = sin_theta / sin_theta_0;

        return Quaternion(
            s1 * q1.w + s2 * q2.w,
            s1 * q1.x + s2 * q2.x,
            s1 * q1.y + s2 * q2.y,
            s1 * q1.z + s2 * q2.z);
    }
};