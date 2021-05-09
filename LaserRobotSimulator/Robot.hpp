#pragma once

#include <vector>
#include <algorithm>

#include "RobotFwd.hpp"

#include "DHParam.hpp"
#include "CoordinateStub.hpp"

#include "Vector3D.hpp"

#include "Rotation.hpp"
#include "Transform.hpp"
#include "Simulator.hpp"

#include "Matrix.hpp"

#include "ForceMomentCouple.hpp"
#include "VelocityCouple.hpp"

#include "ForwardKinematics.hpp"
#include "InverseKinematics.hpp"
#include "Statics.hpp"

#include "Camera.hpp"

#include "RobotPathPlanner.hpp"

enum class SimulationMode { NONE, STATICS, FWD_KINEMATICS, INV_KINEMATICS, PATH_PLAN, VIEW_ALL };

class Robot
{
private:
    /* -- DH Parameters. Governs basically everything else. -- */
    std::vector<DHParam> m_dhParams;

    /* -- Math models for kinematics, statics, and path planning -- */
    ForwardKinematics m_fwdK;
    InverseKinematics *m_invK;
    Statics m_statics;
    RobotPathPlanner *m_planner;

    /* -- Simulation GUI Controls -- */
    SimulationMode m_simulationMode;
    size_t m_idx = 0;

public:
    Robot();
    ~Robot();

    Robot(const Robot &rhs) = delete;

    SimulationMode SimMode() { return m_simulationMode; }
    void SelectSimMode(SimulationMode mode) { m_simulationMode = mode; }

    ForwardKinematics &GetFwdK() { return m_fwdK; }
    InverseKinematics &GetInvK() { return *m_invK; }
    Statics &GetStatics() { return m_statics; }
    RobotPathPlanner &GetPlanner() { return *m_planner; }

    std::vector<DHParam> GetAllDHParams() { return this->m_dhParams; }

    void IncrementIdx() { if (this->m_idx + (size_t)1 < this->m_dhParams.size()) this->m_idx++; }
    void DecrementIdx() { if (this->m_idx > 0) this->m_idx--; }
    void SelectIdx(int idx) { if (idx < this->m_dhParams.size()) this->m_idx = idx; }
    size_t GetIdx() { return this->m_idx; }
    size_t NumJoints() { return this->m_dhParams.size(); }

    void Recompute();
};

class RobotView
{
private:
    Robot *m_robot;
    Camera m_camera;

    void RenderStub(CoordinateStub &stub, bool highlighted, size_t frameNumber, bool zGreen = true);

    void RenderJointVelocity(
        CoordinateStub &stub, double velocity, JointType jointType, int frameNumber);
    void RenderJointStaticReaction(
        CoordinateStub &stub, double reaction, JointType jointType, int frameNumber);

    std::vector<bool> m_shown;

    void ShowJointTable(std::vector<DHParam> dhparams);
    void ShowEEPosition(Vector3D<double> linPos, Vector3D<double> angPos);

    void ShowFwdJointTable() { this->ShowJointTable(m_robot->GetFwdK().DHParams()); }
    void ShowFwdEEPosition() 
    { this->ShowEEPosition(m_robot->GetFwdK().EELinPos(), m_robot->GetFwdK().EEAngPos()); }

    void ShowFwdK();
    void ShowFwdDK();

    void ShowInvJointTable() { this->ShowJointTable(m_robot->GetInvK().GetPose()); }
    void ShowInvEEPosition() 
    { this->ShowEEPosition(m_robot->GetInvK().EELinPos(), m_robot->GetInvK().EEAngPos()); }

    void ShowInvK();
    void ShowInvDK();

    void ShowEEStatics();
    void ShowJointStatics();
    void ShowBaseTransformStatics();

    void ShowStatics();

    void ShowPathPlannerState();
    void ShowPathPlannerP12();
    void ShowPathPlannerTravelTime();
    void ShowPathPlanner();

public:
    RobotView(Robot *robot);
    RobotView(const RobotView &) = delete;

    Camera &Camera() { return m_camera; }

    void ToggleShown(size_t idx);
    void ShowAll() { std::fill(this->m_shown.begin(), this->m_shown.end(), true); }

    void ShowSolo(size_t idx);

    void Poll();
};

class RobotController
{
private:
    Robot *m_robot;
    RobotView *m_view;

    void PollChangeIdx();

    void PollSelectSimMode();

    void PollChangeFwdK();
    void PollChangeFwdDK();

    void PollChangeInvK();
    void PollChangeInvDK();

    void PollChangeCamera();
    void PollToggleShown();

    void PollChangeStatics();

    void PollChangePathPlanner();

public:
    RobotController(Robot *robot, RobotView *view);
    RobotController(const RobotController &) = delete;

    void Poll();
};

