#pragma once

#include "Vector3D.hpp"
#include "ForwardKinematics.hpp"
#include "ForceMomentCouple.hpp"

class Statics
{
private:
    ForwardKinematics *m_fwdK = nullptr;

    std::vector<double> m_qReactions;

    ForceMomentCouple m_eeFMCouple;
    Vector3D<double> m_bEqForce;
    Vector3D<double> m_bEqMoment;

public:
    Statics() = default;
    Statics(ForwardKinematics *fwdK);

    void Recompute();

    void AccEEForce(double deltaMag, double deltaAngX, double deltaAngY);
    void AccEEMoment(double deltaMag, double deltaAngX, double deltaAngY);

    ForceMomentCouple &EEFMCouple() { return m_eeFMCouple; }

    Vector3D<double> BEqForce() { return m_bEqForce; }
    Vector3D<double> BEqMoment() { return m_bEqMoment; }

    std::vector<double> &QReactions() { return m_qReactions; }
};

