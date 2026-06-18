#pragma once
#include <cmath>
class Matrix
{

    double grid[3][3];
    Matrix(double a00, double a01, double a02,
           double a10, double a11, double a12,
           double a20, double a21, double a22)
    {
        grid[0][0] = a00;
        grid[0][1] = a01;
        grid[0][2] = a02;
        grid[1][0] = a10;
        grid[1][1] = a11;
        grid[1][2] = a12;
        grid[2][0] = a20;
        grid[2][1] = a21;
        grid[2][2] = a22;
    }

    static Matrix Rx(double theta)
    {
        double c = std::cos(theta);
        double s = std::sin(theta);
        return Matrix(1, 0, 0, 0, c, -s, 0, s, c);
    }

    static Matrix Ry(double theta)
    {
        double c = std::cos(theta);
        double s = std::sin(theta);
        return Matrix(c, 0, s, 0, 1, 0, -s, 0, c);
    }

    static Matrix
    Rz(double theta)
    {
        double c = std::cos(theta);
        double s = std::sin(theta);
        return Matrix(c, -s, 0, s, c, 0, 0, 0, 1)
    }

    Matrix operator*(const Matrix *other) const
    {

        Matrix result(0, 0, 0, 0, 0, 0, 0, 0, 0);

        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                for (int k = 0; k < 3; ++k)
                {
                    result.grid[i][j] += grid[i][k] * other->grid[k][j];
                }
            }
        }
        return result;
    }

    Vector operator*(const Vector *v) const
    {
        return Vector(
            grid[0][0] * v->x + grid[0][1] * v->y + grid[0][2] * v->z,
            grid[1][0] * v->x + grid[1][1] * v->y + grid[1][2] * v->z,
            grid[2][0] * v->x + grid[2][1] * v->y + grid[2][2] * v->z);
    }

    Matrix transpose() const
    {
        return Matrix(
            grid[0][0], grid[1][0], grid[2][0],
            grid[0][1], grid[1][1], grid[2][1],
            grid[0][2], grid[1][2], grid[2][2]);
    }
}