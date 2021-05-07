#pragma once

#include <vector>

#include "DHParam.hpp"
#include "Matrix.hpp"
#include "JointPath.hpp"
#include "ForwardKinematics.hpp"
#include "Vector3D.hpp"

#define GRAVITY_IN_BASE         (Vector3D<double>(0, 0, -9.80665))
#define DIFFERENTIAL_DELTA      (1e-8)

class Dynamics
{
private:
    std::vector<DHParam> m_dhParams;
    std::vector<JointPath> m_jointPaths;
    std::vector<double> m_torques;

    double KineticAtPose(std::vector<double> qPos, std::vector<double> qVel);
    double PotentialAtPose(std::vector<double> qPos);

    double LagrAtPose(std::vector<double> qPos, std::vector<double> qVel);

    std::vector<double> LagrVsQAtTime(double time);
    std::vector<double> LagrVsQPrimeAtTime(double time);

public:
    Dynamics() = default;
    Dynamics(std::vector<DHParam> dhparams, std::vector<JointPath> jointPaths):
        m_dhParams(dhparams), m_jointPaths(jointPaths) {}

    void RecomputeAtTime(double time);
    std::vector<double> Torques() { return m_torques; }
};

