#pragma once
#include <cmath>
#include <vector>
#include <stdexcept>

class Jacobian;

// general MxN matrix.
//  matrix size dependss on hgowm anyt jointss the roobt has.

class MatrixX
{
public:
    int rows, cols;
    std::vector<double> data;
    MatrixX(int rows, int cols) : rows(rows), cols(cols), data(rows * cols, 0.0) {}

    double &operator()(int r, int c)
    {
        return data[r * cols + c];
    }

    double operator()(int r, int c) const
    {
        return data[r * cols + c];
    }

    static MatrixX identity(int n)
    {
        MatrixX m(n, n);
        for (int i = 0; i < n; i++)
            m(i, i) = 1.0;
        return m;
    }

    MatrixX operator*(const MatrixX &other) const
    {
        if (cols != other.rows)
            throw std::runtime_error("MatrixX multiply: dimension mismatch");

        MatrixX result(rows, other.cols);
        for (int i = 0; i < rows; i++)
        {
            for (int j = 0; j < other.cols; j++)
            {
                double sum = 0.0;
                for (int k = 0; k < cols; k++)
                {
                    sum += (*this)(i, k) * other(k, j);
                }
                result(i, j) = sum;
            }
        }
        return result;
    }

    MatrixX transpose() const
    {
        MatrixX result(cols, rows);
        for (int i = 0; i < rows; i++)
        {
            for (int j = 0; j < cols; j++)
            {
                result(j, i) = (*this)(i, j);
            }
        }
        return result;
    }

    std::vector<double> multiply(const std::vector<double> &v) const
    {
        if (cols != (int)v.size())
            throw std::runtime_error("MatrixX*vector: dimension mismatch");

        std::vector<double> result(rows, 0.0);
        for (int i = 0; i < rows; i++)
        {
            double sum = 0.0;
            for (int j = 0; j < cols; j++)
            {
                sum += (*this)(i, j) * v[j];
            }
            result[i] = sum;
        }
        return result;
    }

    void addDamping(double lambda)
    {
        for (int i = 0; i < rows && i < cols; i++)
            (*this)(i, i) += lambda * lambda;
    }

    MatrixX inverse() const
    {
        if (rows != cols)
            throw std::runtime_error("MatrixX inverse: matrix must be square");

        int n = rows;

        // augmented matrix [A | I] as an n x (2n) MatrixX
        MatrixX aug(n, 2 * n);
        for (int i = 0; i < n; i++)
        {
            for (int j = 0; j < n; j++)
                aug(i, j) = (*this)(i, j);
            aug(i, n + i) = 1.0; // identity on the right half
        }

        for (int col = 0; col < n; col++)
        {

            int pivotRow = col;
            double maxVal = std::abs(aug(col, col));
            for (int r = col + 1; r < n; r++)
            {
                if (std::abs(aug(r, col)) > maxVal)
                {
                    maxVal = std::abs(aug(r, col));
                    pivotRow = r;
                }
            }

            if (maxVal < 1e-12)
                throw std::runtime_error("MatrixX inverse: matrix is singular");

            if (pivotRow != col)
            {
                for (int j = 0; j < 2 * n; j++)
                {
                    double tmp = aug(col, j);
                    aug(col, j) = aug(pivotRow, j);
                    aug(pivotRow, j) = tmp;
                }
            }

            double pivot = aug(col, col);
            for (int j = 0; j < 2 * n; j++)
                aug(col, j) /= pivot;

            for (int r = 0; r < n; r++)
            {
                if (r == col)
                    continue;
                double factor = aug(r, col);
                for (int j = 0; j < 2 * n; j++)
                    aug(r, j) -= factor * aug(col, j);
            }
        }

        // extract the right half -  inverse
        MatrixX result(n, n);
        for (int i = 0; i < n; i++)
            for (int j = 0; j < n; j++)
                result(i, j) = aug(i, n + j);

        return result;
    }

    std::vector<double> solve(const std::vector<double> &b) const
    {
        if (rows != cols)
            throw std::runtime_error("MatrixX solve: matrix must be square");
        if ((int)b.size() != rows)
            throw std::runtime_error("MatrixX solve: RHS size mismatch");

        int n = rows;
        MatrixX aug(n, n + 1);
        for (int i = 0; i < n; i++)
        {
            for (int j = 0; j < n; j++)
                aug(i, j) = (*this)(i, j);
            aug(i, n) = b[i];
        }

        for (int col = 0; col < n; col++)
        {
            int pivotRow = col;
            double maxVal = std::abs(aug(col, col));
            for (int r = col + 1; r < n; r++)
            {
                if (std::abs(aug(r, col)) > maxVal)
                {
                    maxVal = std::abs(aug(r, col));
                    pivotRow = r;
                }
            }

            if (maxVal < 1e-12)
                throw std::runtime_error("MatrixX solve: matrix is singular");

            if (pivotRow != col)
            {
                for (int j = col; j <= n; j++)
                {
                    double tmp = aug(col, j);
                    aug(col, j) = aug(pivotRow, j);
                    aug(pivotRow, j) = tmp;
                }
            }

            double pivot = aug(col, col);
            for (int j = col; j <= n; j++)
                aug(col, j) /= pivot;

            for (int r = 0; r < n; r++)
            {
                if (r == col)
                    continue;
                double factor = aug(r, col);
                for (int j = col; j <= n; j++)
                    aug(r, j) -= factor * aug(col, j);
            }
        }

        std::vector<double> x(n);
        for (int i = 0; i < n; i++)
            x[i] = aug(i, n);
        return x;
    }

    double determinant() const
    {
        if (rows != cols)
            throw std::runtime_error("MatrixX determinant: matrix must be square");

        int n = rows;
        MatrixX aug(n, n);
        for (int i = 0; i < n; i++)
            for (int j = 0; j < n; j++)
                aug(i, j) = (*this)(i, j);

        double det = 1.0;
        for (int col = 0; col < n; col++)
        {
            // partial pivoting
            int pivotRow = col;
            double maxVal = std::abs(aug(col, col));
            for (int r = col + 1; r < n; r++)
            {
                if (std::abs(aug(r, col)) > maxVal)
                {
                    maxVal = std::abs(aug(r, col));
                    pivotRow = r;
                }
            }

            if (maxVal < 1e-12)
                return 0.0; // singular

            // row swap flips sign of determinant
            if (pivotRow != col)
            {
                for (int j = 0; j < n; j++)
                {
                    double tmp = aug(col, j);
                    aug(col, j) = aug(pivotRow, j);
                    aug(pivotRow, j) = tmp;
                }
                det *= -1.0;
            }

            // accumulate pivot into determinant
            det *= aug(col, col);

            // eliminate below
            double pivot = aug(col, col);
            for (int r = col + 1; r < n; r++)
            {
                double factor = aug(r, col) / pivot;
                for (int j = col; j < n; j++)
                    aug(r, j) -= factor * aug(col, j);
            }
        }
        return det;
    }
};