#include "Transform.hpp"
#include <iostream>

Transform::Transform(double theta, double alpha, double a, double d): Transform()
{
    Rotation thetaRotation = Rotation::CreateAboutZ(theta);
    Rotation alphaRotation = Rotation::CreateAboutX(alpha);

    Rotation rotation = Rotation::Multiply(thetaRotation, alphaRotation);

    double dx = a * cos(theta);
    double dy = a * sin(theta);
    double dz = d;

    for (int row = 0; row < 3; row++)
        for (int col = 0; col < 3; col++)
            this->m_data.At(row, col) = rotation.At(row, col);

    this->At(0, 3) = dx;
    this->At(1, 3) = dy;
    this->At(2, 3) = dz;
}

Transform::Transform(Rotation rotation, Vector3D<double> displacement): Transform()
{
    for (int row = 0; row < 3; row++)
        for (int col = 0; col < 3; col++)
            this->At(row, col) = rotation.At(row, col);

    this->At(0, 3) = displacement.x;
    this->At(1, 3) = displacement.y;
    this->At(2, 3) = displacement.z;
}

Vector3D<double> Transform::GetColumn(int col)
{
    return Vector3D<double>(this->At(0, col), this->At(1, col), this->At(2, col));
}

Transform Transform::Inverse()
{
    Transform inverse;
    inverse.m_data = this->m_data.Inverted();
    return inverse;
}

Rotation Transform::GetRotation()
{
    return Rotation(this->m_data.SubMatrix(0, 3, 0, 3));
}

Vector3D<double> Transform::GetDisplacement()
{
    return Vector3D<double>(this->At(0, 3), this->At(1, 3), this->At(2, 3));
}

Transform Transform::Multiply(Transform &lhs, Transform &rhs)
{
    Transform result;
    result.m_data = Matrix::Multiply(lhs.m_data, rhs.m_data);
    return result;
}

std::ostream &operator<<(std::ostream &os, Transform &transform)
{
    for (int row = 0; row < 4; row++)
    {
        os << "[" << transform.At(row, 0);
        os << " " << transform.At(row, 1);
        os << " " << transform.At(row, 2);
        os << " " << transform.At(row, 3);
        os << "]" << std::endl;
    }

    return os;
}
