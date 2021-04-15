#pragma once

#include "RobotFwd.hpp"
#include "Vector3D.hpp"

#include <vector>

class Jacobian
{
private:
    std::vector<double> m_data;
    std::vector<double> Multiply(const std::vector<double> &values);

    size_t GetHeadPosition(size_t row);

    static void SortByHead(Jacobian &lhs, Jacobian &rhs);
    static void SwapRow(Jacobian &lhs, Jacobian &rhs, size_t rowA, size_t rowB);

    static void NormalizeToHead(Jacobian &lhs, Jacobian &rhs, size_t row);

    static void AddRow(Jacobian &lhs, Jacobian &rhs, size_t rowFrom, size_t rowTo, double scaler);
    static void RowReduceDown(Jacobian &lhs, Jacobian &rhs, size_t baseRow);
    static void RowReduceUp(Jacobian &lhs, Jacobian &rhs, size_t baseRow);

public:
    Jacobian();

    void ForwardMultiply(
        const std::vector<double> &jointSpeeds,
        Vector3D<double> &linearOut, 
        Vector3D<double> &angularOut);

    std::vector<double> InverseMultiply(Vector3D<double> linear, Vector3D<double> angular);

    Jacobian Inverted();

    void Set(size_t row, size_t col, double val);
    double Get(size_t row, size_t col);
};

