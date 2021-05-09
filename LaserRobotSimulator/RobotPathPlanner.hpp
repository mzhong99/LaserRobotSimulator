#pragma once

#include "SWTimer.hpp"
#include "Vector3D.hpp"
#include "Dynamics.hpp"
#include "InverseKinematics.hpp"
#include "CoordinateStub.hpp"

#define ACCEPTABLE_INSTABILITY      (1e-1)

enum class RobotPathPlannerState
{ PLAN_P1, PLAN_P2, COMPUTING_PATH_P1, COMPUTING_PATH_P2, DISPLAY_RESULT };

class RobotPathPlanner
{
private:
    std::vector<DHParam> m_dhParams;

    Vector3D<double> m_linP1 = 0;
    Vector3D<double> m_angP1 = 0;
    std::vector<DHParam> m_qPosP1;

    Vector3D<double> m_linP2 = 0;
    Vector3D<double> m_angP2 = 0;
    std::vector<DHParam> m_qPosP2;

    /* Reconstructed whenever path tracing display is currently showing */
    Dynamics m_dynamics;
    InverseKinematics *m_invK;
    ForwardKinematics m_fwdK;

    std::vector<double> m_torques;

    Vector3D<double> m_invTargetLin = 0;
    Vector3D<double> m_invTargetAng = 0;

    Vector3D<double> m_invStepLin = 0;
    Vector3D<double> m_invStepAng = 0;
    int m_stepCounter = 0;

    double m_travelTime = 3.0;

    SWTimer m_showTimer;

    RobotPathPlannerState m_state = RobotPathPlannerState::PLAN_P1;

    void AimInvK(Vector3D<double> eeTargetLin, Vector3D<double> eeTargetAng);
    void IncrementAimInvK();

    void HandleComputePathP1();
    void HandleComputePathP2();
    void HandleDisplayResult();

public:
    RobotPathPlanner(std::vector<DHParam> dhparams): m_dhParams(dhparams)
        { m_invK = new InverseKinematics(dhparams); }
    ~RobotPathPlanner() { delete m_invK; }

    RobotPathPlanner(RobotPathPlanner &rhs) = delete;

    void Recompute();

    void AdvanceState();
    void AccLinPos(Vector3D<double> deltaPos);
    void AccAngPos(Vector3D<double> deltaPos);
    void AccTravelTime(double delta);

    CoordinateStub GetP1Stub() { return CoordinateStub(m_linP1, Rotation()); }
    CoordinateStub GetP2Stub() { return CoordinateStub(m_linP2, Rotation()); }
    RobotPathPlannerState GetState() { return m_state; }

    Vector3D<double> GetP1Ang() { return m_angP1; };
    Vector3D<double> GetP2Ang() { return m_angP2; };

    double TravelTime() { return m_travelTime; }
    std::vector<CoordinateStub> GetInvKApprox() { return m_invK->GetStubs(); }
    std::vector<DHParam> GetInvKTable() { return m_invK->GetPose(); }

    std::vector<CoordinateStub> GetPathPlan() { return m_fwdK.Stubs(); }
    std::vector<DHParam> GetPathPlanTable() { return m_fwdK.DHParams(); }
    std::vector<double> GetQReactions() { return m_torques; }
};

