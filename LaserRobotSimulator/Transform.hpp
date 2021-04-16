#pragma once

#include <vector>
#include <iosfwd>
#include "Rotation.hpp"
#include "Matrix.hpp"

class Transform
{
private:
    Matrix m_data;

public:
    Transform(): m_data(Matrix::Identity(4)) {}
    Transform(double theta, double alpha, double a, double d);
    Transform(Rotation rotation, Vector3D<double> displacement);

    double &At(size_t row, size_t col) { return this->m_data.At(row, col); }
    Vector3D<double> GetColumn(int col);

    friend std::ostream &operator<<(std::ostream &os, Transform &transform);

    Transform Inverse();

    Rotation GetRotation();
    Vector3D<double> GetDisplacement();

    static Transform Multiply(Transform &lhs, Transform &rhs);
};

