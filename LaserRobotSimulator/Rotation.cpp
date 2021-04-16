#include "Rotation.hpp"
#include <cmath>

Rotation::Rotation()
{
    m_data = Matrix::Identity(3);
}

Rotation Rotation::CreateAboutZ(double angle)
{
    Rotation rotation;

    rotation.At(0, 0) = cos(angle);
    rotation.At(0, 1) = -1.0 * sin(angle);
    rotation.At(1, 0) = sin(angle);
    rotation.At(1, 1) = cos(angle);

    return rotation;
}

Rotation Rotation::CreateAboutY(double angle)
{
    Rotation rotation;

    rotation.At(0, 0) =cos(angle);
    rotation.At(0, 2) =-1.0 * sin(angle);
    rotation.At(2, 0) =sin(angle);
    rotation.At(2, 2) =cos(angle);

    return rotation;
}

Rotation Rotation::CreateAboutX(double angle)
{
    Rotation rotation;

    rotation.At(1, 1) = cos(angle);
    rotation.At(1, 2) = -1.0 * sin(angle);
    rotation.At(2, 1) = sin(angle);
    rotation.At(2, 2) = cos(angle);

    return rotation;
}

Rotation Rotation::Multiply(Rotation &lhs, Rotation &rhs)
{
    return Rotation(Matrix::Multiply(lhs.m_data, rhs.m_data));
}

Rotation Rotation::Transpose()
{
    Rotation result;
    result.m_data = result.m_data.Transposed();

    return result;
}

Rotation Rotation::Negate()
{
    Rotation result;

    for (size_t row = 0; row < this->m_data.Rows(); row++)
        for (size_t col = 0; col < this->m_data.Cols(); col++)
            result.m_data.At(row, col) = this->m_data.At(row, col) * -1.0;

    return result;
}

Vector3D<double> Rotation::Rotate(Vector3D<double> p)
{
    std::vector<double> pvec = { p.x, p.y, p.z };
    Matrix pcol = Matrix::CreateCol(pvec);
    Matrix result = Matrix::Multiply(this->m_data, pcol);

    std::vector<double> flattened = result.Flatten();
    return Vector3D<double>(flattened[0], flattened[1], flattened[2]);
}

std::ostream &operator<<(std::ostream &os, Rotation &rotation)
{
    for (int row = 0; row < 3; row++)
    {
        os << "[" << rotation.At(row, 0);
        os << " " << rotation.At(row, 1);
        os << " " << rotation.At(row, 2);
        os << "]" << std::endl;
    }

    return os;
}
