
#include <cmath>
class Vector
{

    // 64 bit
    double x, y, z;

    Vector(double x, double y, double z) : x(x), y(y), z(z) {}

    Vector operator+(const Vector &v) const
    {
        return Vector(x + v.x, y + v.y, z + v.z);
    }

    Vector operator-(const Vector &v) const
    {
        return Vector(x - v.x, y - v.y, z - v.z);
    }

    Vector operator*(double scalar) const
    {
        return Vector(x * scalar, y * scalar, z * scalar);
    }

    // takes two vectrs and returns a single number that is the dot product of the two vectors, (scalar)
    //  measures how much two vectors poitn in the same direction

    double dot(const Vector &v) const
    {
        return x * v.x + y * v.y + z * v.z;
    }

    // two vectrors and reeturns a third vector, perpendicular to both of the vectors
    // right hand rule follows it.
    Vector cross(const Vector &v) const
    {
        return Vector(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x);
    }

    // lenght of vector is magnitude and unit vector(normalized vector) has magnitude of 1,
    double norm() const
    {
        return std::sqrt(x * x + y * y + z * z);
    }
}
