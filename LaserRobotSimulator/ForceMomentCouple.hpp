#pragma once

#include "Rotation.hpp"
#include "Matrix.hpp"

#define DEFAULT_FM_MAGNITUDE    (1.3)

struct FMVector
{
    double XRotation = 0;
    double YRotation = 0;
    double Magnitude = DEFAULT_FM_MAGNITUDE;

    Vector3D<double> Cartesian();
};

class ForceMomentCouple
{
private:
    FMVector m_force;
    FMVector m_moment;

public:
    ForceMomentCouple() {}

    void AccumulateForceXAngle(double delta) { this->m_force.XRotation += delta; }
    void AccumulateForceYAngle(double delta) { this->m_force.YRotation += delta; }
    void AccumulateForceMagnitude(double delta) { this->m_force.Magnitude += delta; }

    void AccumulateMomentXAngle(double delta) { this->m_moment.XRotation += delta; }
    void AccumulateMomentYAngle(double delta) { this->m_moment.YRotation += delta; }
    void AccumulateMomentMagnitude(double delta) { this->m_moment.Magnitude += delta; }

    Matrix AsColumn();
    Vector3D<double> Force() { return m_force.Cartesian(); }
    Vector3D<double> Moment() { return m_moment.Cartesian(); }
};

