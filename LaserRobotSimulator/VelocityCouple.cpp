#include "VelocityCouple.hpp"

Vector3D<double> VelocityVector::Cartesian()
{
    Vector3D<double> unshifted = Vector3D<double>(0, 0, this->Magnitude);

    Rotation aboutX = Rotation::CreateAboutX(this->XRotation);
    Rotation aboutY = Rotation::CreateAboutY(this->YRotation);

    Rotation rotation = Rotation::Multiply(aboutX, aboutY);
    return rotation.Rotate(unshifted);
}

Matrix VelocityCouple::AsColumn()
{
    Matrix answer = Matrix(6, 1);

    Vector3D<double> linear = m_linear.Cartesian();
    Vector3D<double> angular = m_angular.Cartesian();


    answer.At(0, 0) = linear.x;
    answer.At(1, 0) = linear.y;
    answer.At(2, 0) = linear.z;

    answer.At(3, 0) = angular.x;
    answer.At(4, 0) = angular.y;
    answer.At(5, 0) = angular.z;

    return answer;
}
