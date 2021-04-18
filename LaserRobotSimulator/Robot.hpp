#pragma once

#include <vector>
#include <algorithm>

#include "RobotFwd.hpp"

#include "Vector3D.hpp"

#include "Rotation.hpp"
#include "Transform.hpp"
#include "Simulator.hpp"

#include "Matrix.hpp"

#include "ForceMomentCouple.hpp"
#include "VelocityCouple.hpp"
#include "Camera.hpp"

enum class JointType { PRISMATIC, REVOLUTE, NONE };
enum class SimulationMode { NONE, STATICS, DIFFERENTIAL_KINEMATICS, VIEW_ALL };

struct DHParam
{
    double Theta;
    double Alpha;
    double A;
    double D;

    JointType Type;

    DHParam(): 
        Theta(0), Alpha(0), A(0), D(0), Type(JointType::NONE) {}

    DHParam(double theta, double alpha, double a, double d):
        Theta(theta), Alpha(alpha), A(a), D(d), Type(JointType::NONE) {}

    static DHParam CreateRevolute(double alpha, double a, double d);
    static DHParam CreatePrismatic(double theta, double alpha, double a);

    double GetQ();
    void SetQ(double val);
};

struct CoordinateStub
{
    Vector3D<double> Position;
    Rotation Orientation;

    Transform GetTransform() { return Transform(this->Orientation, this->Position); }

    CoordinateStub(): Position(0) {};
    CoordinateStub(Vector3D<double> position, Rotation orientation): 
        Position(position), Orientation(orientation) {};
};

class Robot
{
private:
    std::vector<DHParam> m_dhParams;
    std::vector<CoordinateStub> m_coordinateStubs;
    Vector3D<double> m_eeAngularPosition;

    std::vector<double> m_manualJointSpeeds;

    SimulationMode m_simulationMode;

    int m_idx = 0;

    VelocityCouple m_eeVelocityCouple;
    std::vector<bool> m_isFreeVariable;

    Matrix m_jacobian;
    bool m_simForwards = true;

    ForceMomentCouple m_forceMoment;
    Vector3D<double> m_baseForceEquivalent;
    Vector3D<double> m_baseMomentEquivalent;

    std::vector<double> m_jointForceMoments;

    Vector3D<double> ShiftedEndEffector(size_t qIter);

    void RecomputeCoordinateStubs();
    void RecomputeJacobian();

    void RecomputeStatics();

public:
    Robot();

    SimulationMode SimMode() { return m_simulationMode; }
    void SelectSimMode(SimulationMode mode) { m_simulationMode = mode; }

    size_t NumJoints() { return this->m_dhParams.size(); }
    DHParam &GetJoint() { return m_dhParams[this->m_idx]; }
    CoordinateStub &GetCoordinateStub() { return m_coordinateStubs[this->m_idx]; }

    std::vector<CoordinateStub> GetAllStubs() { return this->m_coordinateStubs; }
    std::vector<DHParam> GetAllDHParams() { return this->m_dhParams; }

    std::vector<double> GetForwardDKResults();
    std::vector<double> GetInverseDKResults();

    void IncrementIdx()
    { if (this->m_idx + (size_t)1 < this->m_dhParams.size()) this->m_idx++; }

    void DecrementIdx()
    { if (this->m_idx > 0) this->m_idx--; }

    void SelectIdx(int idx) 
    { 
        if (idx < this->m_dhParams.size()) 
            this->m_idx = idx; 
    }
    int GetIdx() { return this->m_idx; }

    void Recompute();

    Vector3D<double> GetEndEffectorPosition() { return m_coordinateStubs.back().Position; }
    Vector3D<double> GetEndEffectorAngularPosition() { return m_eeAngularPosition; }
    
    bool IsSimForwards() { return m_simForwards; }
    void ToggleSimForwards() { m_simForwards = !m_simForwards; }

    void AccumulateJointSpeed(double deltaQ) 
    {
        if (this->IsSimForwards() || this->m_isFreeVariable[this->m_idx])
            this->m_manualJointSpeeds[this->m_idx] += deltaQ; 
    }

    void AccumulateEELinearOrientation(double deltaThetaX, double deltaThetaY)
    {
        m_eeVelocityCouple.Linear().AccumulateXAngle(deltaThetaX);
        m_eeVelocityCouple.Linear().AccumulateYAngle(deltaThetaY);
    }

    void AccumulateEELinearMagnitude(double deltaMagnitude)
    { m_eeVelocityCouple.Linear().AccumulateMagnitude(deltaMagnitude); }

    void AccumulateEEAngularMagnitude(double deltaMagnitude)
    { m_eeVelocityCouple.Angular().AccumulateMagnitude(deltaMagnitude); }

    void AccumulateEEAngularOrientation(double deltaThetaX, double deltaThetaY)
    {
        m_eeVelocityCouple.Angular().AccumulateXAngle(deltaThetaX);
        m_eeVelocityCouple.Angular().AccumulateYAngle(deltaThetaY);
    }

    void AccumulateEEForceOrientation(double deltaThetaX, double deltaThetaY)
    {
        this->m_forceMoment.AccumulateForceXAngle(deltaThetaX);
        this->m_forceMoment.AccumulateForceYAngle(deltaThetaY);
    }

    void AccumulateEEForceMagnitude(double deltaMagnitude)
    { this->m_forceMoment.AccumulateForceMagnitude(deltaMagnitude); }

    void AccumulateEEMomentOrientation(double deltaThetaX, double deltaThetaY)
    {
        this->m_forceMoment.AccumulateMomentXAngle(deltaThetaX);
        this->m_forceMoment.AccumulateMomentYAngle(deltaThetaY);
    }

    void AccumulateEEMomentMagnitude(double deltaMagnitude)
    { this->m_forceMoment.AccumulateMomentMagnitude(deltaMagnitude); }

    /* Represented in coordinates of the end effector */
    Matrix GetEEForceMoment();

    VelocityCouple GetEndEffectorVelocity() { return this->m_eeVelocityCouple; }
    std::vector<double> GetManualJointSpeeds() { return this->m_manualJointSpeeds; }

    Transform EndEffectorTransform() { return this->m_coordinateStubs.back().GetTransform(); }
    std::vector<double> JointForceMomentReactions() { return this->m_jointForceMoments; }

    Vector3D<double> BaseForceEquivalent() { return this->m_baseForceEquivalent; }
    Vector3D<double> BaseMomentEquivalent() { return this->m_baseMomentEquivalent; }

    bool IsFreeJoint(size_t idx) { return this->m_isFreeVariable[idx]; };
};

class RobotView
{
private:
    Robot *m_robot;

    Camera m_camera;

    void RenderStub(CoordinateStub &stub, bool highlighted, size_t frameNumber);

    void RenderJointVelocity(
        CoordinateStub &stub, double velocity, JointType jointType, int frameNumber);
    void RenderJointStaticReaction(
        CoordinateStub &stub, double reaction, JointType jointType, int frameNumber);

    std::vector<bool> m_shown;

    void ShowJointTable();

    void ShowEEPosition();

    void ShowForwardsDK();
    void ShowInverseDK();
    void ShowDK();

    void ShowEEStatics();
    void ShowJointStatics();
    void ShowBaseTransformStatics();

    void ShowStatics();

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

    void PollChangeQ();

    void PollSelectSimMode();
    void PollToggleSimForwards();

    void PollChangeForwardsDK();
    void PollChangeInverseDK();

    void PollChangeDK();
    void PollChangeEndEffector();

    void PollChangeCamera();
    void PollToggleShown();

    void PollChangeInverseStatics();

public:
    RobotController(Robot *robot, RobotView *view);
    RobotController(const RobotController &) = delete;

    void Poll();
};

