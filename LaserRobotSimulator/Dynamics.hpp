#pragma once

#include <vector>

#include "DHParam.hpp"
#include "Matrix.hpp"
#include "JointPath.hpp"
#include "ForwardKinematics.hpp"
#include "Vector3D.hpp"

#include "LinearInterpolator.hpp"

#define GRAVITY_IN_BASE         (Vector3D<double>(0, 0, -9.80665))

class Dynamics
{
private:
    std::vector<DHParam> m_dhParams;
    std::vector<JointPath> m_jointPaths;
    std::vector<double> m_torques;
    std::vector<double> m_qPos;
    std::vector<double> m_qVel;

    double m_travelTime = 1.0;

    double KineticAtPose(std::vector<double> qPos, std::vector<double> qVel);
    double PotentialAtPose(std::vector<double> qPos);

    double LagrAtPose(std::vector<double> qPos, std::vector<double> qVel);

    std::vector<double> LagrVsQAtTime(double time);
    std::vector<double> LagrVsQPrimeAtTime(double time);

    std::vector<LinearInterpolator> m_qPosInterpolated;
    std::vector<LinearInterpolator> m_qVelInterpolated;
    std::vector<LinearInterpolator> m_torquesInterpolated;

    void RecomputeAtTime(double time);
    std::vector<double> Torques() { return m_torques; }
    std::vector<double> QPos() { return m_qPos; }
    std::vector<double> QVel() { return m_qVel; }

    void Precompute();

public:
    Dynamics() = default;
    Dynamics(std::vector<DHParam> dhparams, std::vector<JointPath> jointPaths, double travelTime):
        m_dhParams(dhparams), m_jointPaths(jointPaths), m_travelTime(travelTime)
        { this->Precompute(); }

    std::vector<double> TorquesAtTime(double time);
    std::vector<double> QPosAtTime(double time);
    std::vector<double> QVelAtTime(double time);
};

