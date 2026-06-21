#include <iostream>
#include "../math/vectors.hpp"
#include "../math/matrix.hpp"
#include "../math/transform.hpp"

void runVectorTransformTest()
{
    std::cout << "Vector and Transform Test" << std::endl;

    Vector a(1, 0, 0);
    Vector b(0, 1, 0);
    Vector sum = a + b;
    std::cout << "a + b = " << sum << std::endl;

    double angle = 1.5707963267948966; // pi/2
    Matrix rz90 = Matrix::Rz(angle);

    Vector rotated = rz90 * a;
    std::cout << "Rz(90) * (1,0,0) = " << rotated << std::endl;

    Transform t(rz90, Vector(2, 0, 0));
    Vector result = t.transformPoint(a);
    std::cout << "Transform applied to (1,0,0) = " << result << std::endl;
    std::cout << std::endl;
}
