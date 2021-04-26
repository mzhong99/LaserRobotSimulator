#include "Statics.hpp"

Statics::Statics(ForwardKinematics *fwdK) 
{
    m_fwdK = fwdK;
    m_qReactions.resize(fwdK->Stubs().size(), 0);
}
void Statics::Recompute()
{
    Matrix jacobianT = m_fwdK->Jacobian().Transposed().Multiply(-1.0);
    Matrix eeFMCol = m_eeFMCouple.AsColumn();

    m_qReactions = Matrix::Multiply(jacobianT, eeFMCol).Flatten();

    /* Base frame equivalent force */
    Vector3D<double> posEEToBase = m_fwdK->EELinPos();
    m_bEqForce = m_eeFMCouple.Force();

    Vector3D<double> momentAboutEEInBase = m_eeFMCouple.Moment();
    Vector3D<double> momentDelta = Vector3D<double>::Cross(posEEToBase, m_bEqForce);
    m_bEqMoment = momentAboutEEInBase - momentDelta;
}

void Statics::AccEEForce(double deltaMag, double deltaAngX, double deltaAngY)
{
    m_eeFMCouple.AccumulateForceMagnitude(deltaMag);
    m_eeFMCouple.AccumulateForceXAngle(deltaAngX);
    m_eeFMCouple.AccumulateForceYAngle(deltaAngY);
}

void Statics::AccEEMoment(double deltaMag, double deltaAngX, double deltaAngY)
{
    m_eeFMCouple.AccumulateMomentMagnitude(deltaMag);
    m_eeFMCouple.AccumulateMomentXAngle(deltaAngX);
    m_eeFMCouple.AccumulateMomentYAngle(deltaAngY);
}
