#pragma once
#include <cmath>
#include "vectors.hpp"

// RX,RY,RZ are rotation matrices, they rotate a vector around the X,Y,Z axis respectively.
// Matrix * Matrix, chain multiple rotations together
// Matrix * Vctor, apply a rotation to a point or direction
// transpose, undo a rotation very cheaply, since this is the inverse of a matrix.
class Matrix
{
public:
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

    // Rotate around the X acis, so Y and Z rotate eachj other, x in this case stays 1,0,0
    static Matrix Rx(double theta)
    {
        double c = std::cos(theta);
        double s = std::sin(theta);
        return Matrix(1, 0, 0, 0, c, -s, 0, s, c);
    }
    // Rotate around the Y axis, so X and Z rotate each other, Y in this case stays 0,1,0
    static Matrix Ry(double theta)
    {
        double c = std::cos(theta);
        double s = std::sin(theta);
        return Matrix(c, 0, s, 0, 1, 0, -s, 0, c);
    }

    // Rotate around the Z axis, so X and Y rotate each other, Z in this case stays 0,0,1
    static Matrix Rz(double theta)
    {
        double c = std::cos(theta);
        double s = std::sin(theta);
        return Matrix(c, -s, 0, s, c, 0, 0, 0, 1);
    }

    static Matrix identity()
    {
        return Matrix(1, 0, 0, 0, 1, 0, 0, 0, 1);
    }

    // you compute two rtoations into one, it gives a single matrix that does both rotations at once.
    // each element is the dot product of a row from the left matrix with a column from the right matrix.
    // you can chain joint rotations together, joint 1 has rotation R1 and joint 2 has rotation R2 relative to joint 1.
    // Total rotation from base to the end of joint 2 is R1 * R2, you keep muiltiplying as you traverse the chain.
    // Walking from base otuward multiplying rotation matrices at each joint.
    Matrix operator*(const Matrix &other) const
    {

        Matrix result(0, 0, 0, 0, 0, 0, 0, 0, 0);

        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                for (int k = 0; k < 3; ++k)
                {
                    result.grid[i][j] += grid[i][k] * other.grid[k][j];
                }
            }
        }
        return result;
    }

    // takes a vector expressed in one frame and expresses it into another frame.
    //  Vector point along X, rotated 90 dfegrees around Z, now points to Y.
    Vector operator*(const Vector &v) const
    {
        return Vector(
            grid[0][0] * v.x + grid[0][1] * v.y + grid[0][2] * v.z,
            grid[1][0] * v.x + grid[1][1] * v.y + grid[1][2] * v.z,
            grid[2][0] * v.x + grid[2][1] * v.y + grid[2][2] * v.z);
    }

    // flips matrix along diagnal, so rows become columns, and columns become rows, so if you have a 3x3 matrix, the first row becomes the first column, the second row becomes the second column, and the third row becomes the third column.
    // how one coordinate frame relates to another, if  R rotates from FRAME A to FRAME B, then R^T describes the rotation from FRAME B to FRAME A.
    // transpose is the inverse of a mmatrix.
    Matrix transpose() const
    {
        return Matrix(
            grid[0][0], grid[1][0], grid[2][0],
            grid[0][1], grid[1][1], grid[2][1],
            grid[0][2], grid[1][2], grid[2][2]);
    }
};