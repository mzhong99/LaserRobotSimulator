#include "ForceMomentCouple.hpp"

Vector3D<double> FMVector::Cartesian()
{
    Vector3D<double> unshifted = Vector3D<double>(0, 0, this->Magnitude);

    Rotation aboutX = Rotation::CreateAboutX(this->XRotation);
    Rotation aboutY = Rotation::CreateAboutY(this->YRotation);

    Rotation rotation = Rotation::Multiply(aboutX, aboutY);
    return rotation.Rotate(unshifted);
}

Matrix ForceMomentCouple::AsColumn()
{
    Vector3D<double> force = m_force.Cartesian();
    Vector3D<double> moment = m_moment.Cartesian();

    Matrix answer = Matrix(6, 1);

    answer.At(0, 0) = force.x;
    answer.At(1, 0) = force.y;
    answer.At(2, 0) = force.z;

    answer.At(3, 0) = moment.x;
    answer.At(4, 0) = moment.y;
    answer.At(5, 0) = moment.z;

    return answer;
}
