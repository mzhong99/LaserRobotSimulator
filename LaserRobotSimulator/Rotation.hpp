#pragma once

#include <vector>
#include <iosfwd>

#include "Vector3D.hpp"
#include "Matrix.hpp"

class Rotation
{
private:
    Matrix m_data;

public:
    static Rotation CreateAboutZ(double angle);
    static Rotation CreateAboutY(double angle);
    static Rotation CreateAboutX(double angle);

    Rotation();
    Rotation(Matrix matrix): m_data(matrix.SubMatrix(0, 3, 0, 3)) {}

    static Rotation Multiply(Rotation &lhs, Rotation &rhs);

    Rotation Transpose();
    Rotation Negate();
    Vector3D<double> Rotate(Vector3D<double> p);

    Vector3D<double> GetColumn(size_t col)
    { return Vector3D<double>(m_data.At(0, col), m_data.At(1, col), m_data.At(2, col)); }

    double &At(size_t row, size_t col) { return this->m_data.At(row, col); }

    friend std::ostream &operator<<(std::ostream &os, Rotation &transform);
};

