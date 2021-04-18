#pragma once

#include "Rotation.hpp"
#include "Matrix.hpp"

#define DEFAULT_VELOCITY_MAGNITUDE  (1.5)

struct VelocityVector
{
    double XRotation = 0;
    double YRotation = 0;
    double Magnitude = DEFAULT_VELOCITY_MAGNITUDE;

    VelocityVector() = default;
    VelocityVector(double xrot, double yrot, double mag): 
        XRotation(xrot), YRotation(yrot), Magnitude(mag) {}

    void AccumulateXAngle(double delta) { XRotation += delta; }
    void AccumulateYAngle(double delta) { YRotation += delta; }
    void AccumulateMagnitude(double delta) { Magnitude += delta; }

    Vector3D<double> Cartesian();

};

class VelocityCouple
{
private:
    VelocityVector m_linear;
    VelocityVector m_angular;

public:
    VelocityCouple() {}
    
    VelocityVector &Linear() { return m_linear; }
    VelocityVector &Angular() { return m_angular; }

    Matrix AsColumn();
};

