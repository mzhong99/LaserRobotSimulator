#pragma once

#include <vector>
#include <iosfwd>
#include "Rotation.hpp"

class Transform
{
private:
    std::vector<double> m_data;

public:
    Transform();
    Transform(double theta, double alpha, double a, double d);
    Transform(Rotation rotation, Vector3D<double> displacement);

    double Get(int row, int col);
    void Set(int row, int col, double val);

    Vector3D<double> GetColumn(int col);

    friend std::ostream &operator<<(std::ostream &os, Transform &transform);

    Transform Inverse();

    Rotation GetRotation();
    Vector3D<double> GetDisplacement();

    static Transform Multiply(Transform &lhs, Transform &rhs);
};

