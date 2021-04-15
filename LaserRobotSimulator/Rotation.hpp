#pragma once

#include <vector>
#include <iosfwd>

#include "Vector3D.hpp"

class Rotation
{
private:
    std::vector<double> m_data;

public:
    static Rotation CreateAboutZ(double angle);
    static Rotation CreateAboutY(double angle);
    static Rotation CreateAboutX(double angle);

    Rotation();
    static Rotation Multiply(Rotation &lhs, Rotation &rhs);

    Rotation Transpose();
    Rotation Negate();
    Vector3D<double> Rotate(Vector3D<double> p);

    void Set(int row, int col, double val);
    double Get(int row, int col);

    friend std::ostream &operator<<(std::ostream &os, Rotation &transform);
};

