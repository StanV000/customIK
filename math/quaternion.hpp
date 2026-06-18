
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
        w_res = w * q.w - x * q.x - y * q.y - z * q.x 
        x_res = w * q.x + x * q.w + y * q.z - z * q.y
        y_res = w * q.y - x * q.z + y * q.w + z * q.x
        z_res = w * q.z + x * q.y - y * q.x + z * q.w
        return Quaternion(w_res, x_res, y_res, z_res);
    }       

    conjugate() const
    {
        return Quaternion(w, -x, -y, -z);
    }


    //TOOD see the optimized verson by using  vector expanasion formula, can break this into 15 multiplications and additions.
    //TODO
    rotate(Vector v) const
    {
        Quaternion qv(0, v.x, v.y ,v.z);
        Quaternion q_conj = conjugate();
        Quaternion q_rotated = (*this) * qv * q_conj;
        return Vector(q_rotated.x, q_rotated.y, q_rotated.z);
    }

    //divide by zero?, check for it later if problem occurs  could try usign 1.0 / sqrt (faster tahn dividing several times? )
    normalize() const
    {
        double normalizaed =  std::sqrt(w*w + x*x + y*y + z*z);
        return Quaternion(w/normalizaed, x/normalizaed, y/normalizaed, z/normalizaed);
    }

    toMatrix() const
    {
        return Matrix(
            1-2*(y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y), 0,
            2*(x*y + w*z), 1-2*(x*x + z*z), 2*(y*z - w*x), 0,
            2*(x*z - w*y), 2*(y*z + w*x), 1-2*(x*x + y*y), 0,
            0, 0, 0, 1
        )
    }

    static Quaternion slerp(const Quaternion &q1, const Quaternion &q2, double t) const
    {
        double dot = q1.w * q2.w + q1.x * q2.x + q1.y * q2.y + q1.z * q2.z;

        if (dot < 0.0f) {
            q2 = Quaternion(-q2.w, -q2.x, -q2.y, -q2.z);
            dot = -dot;
        }

        const double DOT_THRESHOLD = 0.9995;
        if (dot > DOT_THRESHOLD) {
            Quaternion result = Quaternion(
                q1.w + t * (q2.w - q1.w),
                q1.x + t * (q2.x - q1.x),
                q1.y + t * (q2.y - q1.y),
                q1.z + t * (q2.z - q1.z)
            );
            return result.normalize();
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
            s1 * q1.z + s2 * q2.z
        );

    }
}