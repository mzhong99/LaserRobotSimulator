#include "Transform.hpp"
#include <iostream>

Transform::Transform()
{
    m_data.resize(16);

    for (int i = 0; i < 16; i++)
        m_data[i] = 0;

    for (int i = 0; i < 4; i++)
        this->Set(i, i, 1);
}

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
            this->Set(row, col, rotation.Get(row, col));

    this->Set(0, 3, dx);
    this->Set(1, 3, dy);
    this->Set(2, 3, dz);
}

Transform::Transform(Rotation rotation, Vector3D<double> displacement): Transform()
{
    for (int row = 0; row < 3; row++)
        for (int col = 0; col < 3; col++)
            this->Set(row, col, rotation.Get(row, col));

    this->Set(0, 3, displacement.x);
    this->Set(1, 3, displacement.y);
    this->Set(2, 3, displacement.z);
}

Vector3D<double> Transform::GetColumn(int col)
{
    Vector3D<double> column;

    column.x = this->Get(0, col);
    column.y = this->Get(1, col);
    column.z = this->Get(2, col);

    return column;
}

void Transform::Set(int row, int col, double val)
{
    m_data[(row * 4) + col] = val;
}

double Transform::Get(int row, int col)
{
    return m_data[(row * 4) + col];
}

Transform Transform::Inverse()
{
    Transform inverse;

    Rotation rotation;
    for (int row = 0; row < 3; row++)
        for (int col = 0; col < 3; col++)
            rotation.Set(row, col, this->Get(row, col));

    Vector3D<double> d = Vector3D<double>(this->Get(0, 3), this->Get(1, 3), this->Get(2, 3));

    Rotation transposedRotation = rotation.Transpose();
    Vector3D<double> dChanged = transposedRotation.Negate().Rotate(d);

    for (int row = 0; row < 3; row++)
        for (int col = 0; col < 3; col++)
            inverse.Set(row, col, transposedRotation.Get(row, col));

    inverse.Set(0, 3, dChanged.x);
    inverse.Set(1, 3, dChanged.y);
    inverse.Set(2, 3, dChanged.z);

    return inverse;
}

Rotation Transform::GetRotation()
{
    Rotation rotation;

    for (int row = 0; row < 3; row++)
        for (int col = 0; col < 3; col++)
            rotation.Set(row, col, this->Get(row, col));

    return rotation;
}

Vector3D<double> Transform::GetDisplacement()
{
    return Vector3D<double>(this->Get(0, 3), this->Get(1, 3), this->Get(2, 3));
}

Transform Transform::Multiply(Transform &lhs, Transform &rhs)
{
    Transform result;

    for (int row = 0; row < 4; row++)
    {
        for (int col = 0; col < 4; col++)
        {
            double sum = 0;

            for (int i = 0; i < 4; i++)
                sum += lhs.Get(row, i) * rhs.Get(i, col);

            result.Set(row, col, sum);
        }
    }

    return result;
}

std::ostream &operator<<(std::ostream &os, Transform &transform)
{
    for (int row = 0; row < 4; row++)
    {
        os << "[" << transform.Get(row, 0);
        os << " " << transform.Get(row, 1);
        os << " " << transform.Get(row, 2);
        os << " " << transform.Get(row, 3);
        os << "]" << std::endl;
    }

    return os;
}
