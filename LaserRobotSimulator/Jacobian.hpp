#pragma once

#include "RobotFwd.hpp"
#include "Vector3D.hpp"
#include "Matrix.hpp"

#include <vector>
#include <string>

class Jacobian
{
private:
    Matrix m_data;

public:
    Jacobian();

    void ForwardMultiply(
        const std::vector<double> &jointSpeeds,
        Vector3D<double> &linearOut, 
        Vector3D<double> &angularOut);

    std::vector<double> Multiply(const std::vector<double> &values);
    std::vector<double> InverseMultiply(Vector3D<double> linear, Vector3D<double> angular);

    /** Given a vector of end effector velocities, computes the corresponding joint velocities */
    std::vector<double> LeastSquaresMultiply(std::vector<double> jointVelocity);

    Jacobian Inverted();
    Jacobian Transposed();

    static Jacobian Multiply(Jacobian &lhs, Jacobian &rhs);
};

