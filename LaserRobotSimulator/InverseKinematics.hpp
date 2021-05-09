#pragma once

#include <vector>
#include <algorithm>
#include <numeric>

#include <thread>

#include "Vector3D.hpp"
#include "Matrix.hpp"
#include "Semaphore.hpp"

#include "VelocityCouple.hpp"

#include "RobotFwd.hpp"
#include "ForwardKinematics.hpp"

#include "SWTimer.hpp"

class InverseKinematics
{
private:
    /* ------------------------------------------------------------------------------------------ */
    /* Internal data - outsiders should never touch any of these members for any reason.          */
    /* ------------------------------------------------------------------------------------------ */
    std::atomic_bool m_killswitch;
    std::thread m_worker;

    SWTimer m_deltaTimer;
    double m_deltaTime;

    ForwardKinematics m_fwdK;

    std::vector<DHParam> m_dhParams;
    std::vector<double> m_qPrime;

    Vector3D<double> m_eeTargetLinPos;
    Vector3D<double> m_eeTargetAngPos;

    Vector3D<double> m_linError = 0;
    Vector3D<double> m_angError = 0;

    Vector3D<double> m_linIntError = 0;
    Vector3D<double> m_angIntError = 0;

    Vector3D<double> m_linDifError = 0;
    Vector3D<double> m_angDifError = 0;

    VelocityCouple m_eeVel;

    /* ------------------------------------------------------------------------------------------ */
    /* Output lock phase - outsiders only read this data with getters.                            */
    /* ------------------------------------------------------------------------------------------ */
    std::mutex m_outputLock;
    std::vector<DHParam> m_oPose;
    std::vector<CoordinateStub> m_oStubs;
    std::vector<double> m_oQPrime;

    Vector3D<double> m_oEELinPos;
    Vector3D<double> m_oEEAngPos;
    Transform m_oTransformEEtoBase;

    double m_oInstability;

    VelocityCouple m_oEEVel;

    /* ------------------------------------------------------------------------------------------ */
    /* Input lock phase - outsiders only write this data with setters.                            */
    /* ------------------------------------------------------------------------------------------ */
    std::mutex m_inputLock;
    bool m_inputChanged;

    Vector3D<double> m_iEELinPos;
    Vector3D<double> m_iEEAngPos;

    VelocityCouple m_iEEVel;

    /* ------------------------------------------------------------------------------------------ */
    /* MEMBER FUNCTIONS                                                                           */
    /* ------------------------------------------------------------------------------------------ */
    std::vector<double> EEVelToJointVel(Matrix eeVel);
    void RefreshDeltaTime();

    void CopyInputs();

    void RecomputeDistanceCorrection();
    void NewtonApproximateJoints();
    void RecomputeInvDK() { m_qPrime = this->EEVelToJointVel(m_eeVel.AsColumn()); }

    void ReportAnswer();

    void TF_ComputeJointPosition();

public:
    InverseKinematics(std::vector<DHParam> params);
    ~InverseKinematics();
    InverseKinematics(const InverseKinematics &rhs) = delete;

    void AccTargetEEPos(Vector3D<double> eeLinDelta, Vector3D<double> eeAngDelta);
    void AccTargetEELinVel(double deltaMag, double deltaAngX, double deltaAngY);
    void AccTargetEEAngVel(double deltaMag, double deltaAngX, double deltaAngY);

    void SetTargetEEPos(Vector3D<double> eeLinPos, Vector3D<double> eeAngPos);

    std::vector<CoordinateStub> GetStubs();
    std::vector<DHParam> GetPose();
    std::vector<double> GetQPrime();

    Vector3D<double> EELinPos();
    Vector3D<double> EEAngPos();

    Vector3D<double> EELinVel();
    Vector3D<double> EEAngVel();

    double Instability();

    Transform TransformEEToBase();
};

