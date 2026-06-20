#include <iostream>
#include "math/vectors.hpp"
#include "math/matrix.hpp"
#include "math/transform.hpp"

int main()
{

    Vector a(1, 0, 0);
    Vector b(0, 1, 0);
    Vector sum = a + b;
    std::cout << "a + b = " << sum << std::endl;

    // build a 90 degree rotation around Z
    double angle = 1.5707963267948966; // pi/2, 90 degrees in radians
    Matrix rz90 = Matrix::Rz(angle);

    //  rotate (1,0,0) by 90 degrees around Z should be (0,1,0)
    Vector rotated = rz90 * a;
    std::cout << "Rz(90) * (1,0,0) = " << rotated << std::endl;

    // 4. build a transform: rotate 90 around Z, then shift by (2,0,0)
    Transform t(rz90, Vector(2, 0, 0));

    // (1,0,0)
    // (1,0,0) by 90 to (0,1,0) then (2,0,0) to (2,1,0)
    Vector result = t.transformPoint(a);
    std::cout << "Transform applied to (1,0,0) = " << result << std::endl;

    return 0;
}