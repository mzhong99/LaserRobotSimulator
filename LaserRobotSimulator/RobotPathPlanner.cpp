#include "RobotPathPlanner.hpp"

#define NSTEPS      (16)

void RobotPathPlanner::AimInvK(Vector3D<double> eeTargetLin, Vector3D<double> eeTargetAng)
{
    m_invStepLin = (eeTargetLin - m_invK->EELinPos()) * (1.0 / NSTEPS);
    m_invStepAng = (eeTargetAng - m_invK->EEAngPos()) * (1.0 / NSTEPS);

    m_invTargetLin = m_invK->EELinPos() + m_invStepLin;
    m_invTargetAng = m_invK->EEAngPos() + m_invStepAng;
    
    m_invK->SetTargetEEPos(m_invTargetLin, m_invTargetAng);
    m_stepCounter = 0;
}

void RobotPathPlanner::IncrementAimInvK()
{
    m_invTargetLin += m_invStepLin;
    m_invTargetAng += m_invStepAng;

    m_invK->SetTargetEEPos(m_invTargetLin, m_invTargetAng);
    m_stepCounter++;
}

void RobotPathPlanner::HandleComputePathP1()
{
    std::cout << "Instability [P1]: " << m_invK->Instability() << std::endl;
    if (m_invK->Instability() < ACCEPTABLE_INSTABILITY)
    {
        if (m_stepCounter + 1 == NSTEPS)
        {
            m_qPosP1 = m_invK->GetPose();
            this->AimInvK(m_linP2, m_angP2);
            m_state = RobotPathPlannerState::COMPUTING_PATH_P2;
        }
        else
            this->IncrementAimInvK();
    }
}

void RobotPathPlanner::HandleComputePathP2()
{
    std::cout << "Instability [P2]: " << m_invK->Instability() << std::endl;
    if (m_invK->Instability() < ACCEPTABLE_INSTABILITY)
    {
        if (m_stepCounter + 1 == NSTEPS)
        {
            m_qPosP2 = m_invK->GetPose();

            std::vector<JointPath> jointPaths;

            for (size_t i = 0; i < m_dhParams.size(); i++)
                jointPaths.emplace_back(m_qPosP1[i].GetQ(), m_qPosP2[i].GetQ(), m_travelTime);

            m_dynamics = Dynamics(m_dhParams, jointPaths, m_travelTime);
            m_state = RobotPathPlannerState::DISPLAY_RESULT;

            m_fwdK = ForwardKinematics(m_dhParams);
            m_showTimer = SWTimer(m_travelTime);
            m_showTimer.Start();

            this->HandleDisplayResult();
        }
        else
            this->IncrementAimInvK();
    }
}

void RobotPathPlanner::HandleDisplayResult()
{
    if (m_showTimer.Expired())
        m_showTimer.Start();

    std::vector<double> pose = m_dynamics.QPosAtTime(m_showTimer.ElapsedSeconds());
    m_torques = m_dynamics.TorquesAtTime(m_showTimer.ElapsedSeconds());

    for (size_t i = 0; i < pose.size(); i++)
        m_fwdK.SetQ(i, pose[i]);

    m_fwdK.Recompute();
}

void RobotPathPlanner::Recompute()
{
    switch (m_state)
    {
        case RobotPathPlannerState::COMPUTING_PATH_P1:
            this->HandleComputePathP1();
            break;

        case RobotPathPlannerState::COMPUTING_PATH_P2:
            this->HandleComputePathP2();
            break;

        case RobotPathPlannerState::DISPLAY_RESULT:
            this->HandleDisplayResult();
            break;

        default:
            break;
    }
}

void RobotPathPlanner::AccLinPos(Vector3D<double> deltaPos)
{
    switch (m_state)
    {
        case RobotPathPlannerState::PLAN_P1:
            m_linP1 += deltaPos;
            break;

        case RobotPathPlannerState::PLAN_P2:
            m_linP2 += deltaPos;
            break;

        default:
            break;
    }
}

void RobotPathPlanner::AccAngPos(Vector3D<double> deltaPos)
{
    switch (m_state)
    {
        case RobotPathPlannerState::PLAN_P1:
            m_angP1 += deltaPos;
            break;

        case RobotPathPlannerState::PLAN_P2:
            m_angP2 += deltaPos;
            break;

        default:
            break;
    }
}

void RobotPathPlanner::AccTravelTime(double delta) 
{
    if (m_state == RobotPathPlannerState::PLAN_P1 || m_state == RobotPathPlannerState::PLAN_P2)
        m_travelTime += delta;
}

void RobotPathPlanner::AdvanceState()
{
    switch (m_state)
    {
        case RobotPathPlannerState::PLAN_P1:
            m_state = RobotPathPlannerState::PLAN_P2;
            break;

        case RobotPathPlannerState::PLAN_P2:
            this->AimInvK(m_linP1, m_angP1);
            m_state = RobotPathPlannerState::COMPUTING_PATH_P1;
            break;

        case RobotPathPlannerState::DISPLAY_RESULT:
            m_state = RobotPathPlannerState::PLAN_P1;
            break;

        default:
            break;
    }
}

