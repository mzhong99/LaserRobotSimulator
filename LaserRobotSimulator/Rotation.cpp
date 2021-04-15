#include "Rotation.hpp"
#include <cmath>

Rotation::Rotation()
{
    m_data.resize(9);
    
    for (int i = 0; i < 9; i++)
        m_data[i] = 0;

    for (int i = 0; i < 3; i++)
        this->Set(i, i, 1);
}

Rotation Rotation::CreateAboutZ(double angle)
{
    Rotation rotation;

    rotation.Set(0, 0, cos(angle));
    rotation.Set(0, 1, -1.0 * sin(angle));
    rotation.Set(1, 0, sin(angle));
    rotation.Set(1, 1, cos(angle));

    return rotation;
}

Rotation Rotation::CreateAboutY(double angle)
{
    Rotation rotation;

    rotation.Set(0, 0, cos(angle));
    rotation.Set(0, 2, -1.0 * sin(angle));
    rotation.Set(2, 0, sin(angle));
    rotation.Set(2, 2, cos(angle));

    return rotation;
}

Rotation Rotation::CreateAboutX(double angle)
{
    Rotation rotation;

    rotation.Set(1, 1, cos(angle));
    rotation.Set(1, 2, -1.0 * sin(angle));
    rotation.Set(2, 1, sin(angle));
    rotation.Set(2, 2, cos(angle));

    return rotation;
}

Rotation Rotation::Multiply(Rotation &lhs, Rotation &rhs)
{
    Rotation result;

    for (int row = 0; row < 3; row++)
    {
        for (int col = 0; col < 3; col++)
        {
            double sum = 0;

            for (int i = 0; i < 3; i++)
                sum += lhs.Get(row, i) * rhs.Get(i, col);

            result.Set(row, col, sum);
        }
    }

    return result;
}

Rotation Rotation::Transpose()
{
    Rotation result;
    for (int row = 0; row < 3; row++)
        for (int col = 0; col < 3; col++)
            result.Set(row, col, this->Get(col, row));

    return result;
}

Rotation Rotation::Negate()
{
    Rotation result;

    for (int i = 0; i < 9; i++)
        result.m_data[i] = -1.0 * this->m_data[i];

    return result;
}

Vector3D<double> Rotation::Rotate(Vector3D<double> p)
{
    Vector3D<double> result = Vector3D<double>();

    result.x = (this->Get(0, 0) * p.x) + (this->Get(0, 1) * p.y) + (this->Get(0, 2) * p.z);
    result.y = (this->Get(1, 0) * p.x) + (this->Get(1, 1) * p.y) + (this->Get(1, 2) * p.z);
    result.z = (this->Get(2, 0) * p.x) + (this->Get(2, 1) * p.y) + (this->Get(2, 2) * p.z);

    return result;
}

void Rotation::Set(int row, int col, double val)
{
    m_data[(row * 3) + col] = val;
}

double Rotation::Get(int row, int col)
{
    return m_data[(row * 3) + col];
}

std::ostream &operator<<(std::ostream &os, Rotation &rotation)
{
    for (int row = 0; row < 3; row++)
    {
        os << "[" << rotation.Get(row, 0);
        os << " " << rotation.Get(row, 1);
        os << " " << rotation.Get(row, 2);
        os << "]" << std::endl;
    }

    return os;
}
