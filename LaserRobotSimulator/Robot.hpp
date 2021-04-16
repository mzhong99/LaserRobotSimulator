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

enum class JointType { PRISMATIC, REVOLUTE, NONE };

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
    Vector3D<double> AngularPosition();

    CoordinateStub(): Position(0) {};
    CoordinateStub(Vector3D<double> position, Rotation orientation): 
        Position(position), Orientation(orientation) {};
};

class Robot
{
private:
    std::vector<DHParam> m_dhParams;
    std::vector<CoordinateStub> m_coordinateStubs;
    std::vector<double> m_manualJointSpeeds;

    int m_idx = 0;

    std::vector<double> m_manualEndEffectorVelocity;
    std::vector<bool> m_isFreeVariable;

    Matrix m_jacobian;
    bool m_dkForwards = true;

    ForceMomentCouple m_forceMoment;
    Vector3D<double> m_baseForceEquivalent;
    Vector3D<double> m_baseMomentEquivalent;

    std::vector<double> m_jointForceMoments;

    void ShiftedEndEffector(
        size_t qIter, 
        Vector3D<double> &linearOut, 
        Vector3D<double> &angularOut);

    std::vector<double> GetForwardDKResults();
    std::vector<double> GetInverseDKResults();

    void RecomputeCoordinateStubs();
    void RecomputeJacobian();
    void RecomputeStatics();

public:
    Robot();

    DHParam &GetJoint() { return m_dhParams[this->m_idx]; }
    CoordinateStub &GetCoordinateStub() { return m_coordinateStubs[this->m_idx]; }

    std::vector<CoordinateStub> GetAllStubs() { return this->m_coordinateStubs; }
    std::vector<DHParam> GetAllDHParams() { return this->m_dhParams; }

    std::vector<double> GetDKResults();

    void IncrementIdx()
    { if (this->m_idx + (size_t)1 < this->m_dhParams.size()) this->m_idx++; }

    void DecrementIdx()
    { if (this->m_idx > 0) this->m_idx--; }

    void SelectIdx(int idx) { this->m_idx = idx; }
    int GetIdx() { return this->m_idx; }

    void Recompute();

    Vector3D<double> GetEndEffectorPosition() { return m_coordinateStubs.back().Position; }
    
    bool IsDKForwards() { return m_dkForwards; }
    void ToggleDKForwards() { m_dkForwards = !m_dkForwards; }

    void AccumulateJointSpeed(double deltaQ) 
    { 
        if (this->IsDKForwards() || this->m_isFreeVariable[this->m_idx])
            this->m_manualJointSpeeds[this->m_idx] += deltaQ; 
    }

    void AccumulateEndEffectorVelocity(double deltaE);

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

    std::vector<double> GetEndEffectorVelocity() { return this->m_manualEndEffectorVelocity; }
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

    Vector2D<double> m_screenOffset2D;

    double m_yaw;
    double m_pitch;

    double m_zoomFactor;

    void RenderStub(CoordinateStub &stub, bool highlighted, size_t frameNumber);
    void RenderJointStaticReaction(
        CoordinateStub &stub, double reaction, JointType jointType, int frameNumber);

    bool m_showStatics;
    std::vector<bool> m_shown;

    Vector2D<double> WorldToScreen(Vector3D<double> worldVector);

    Rotation GetSphericalRotation();

    void ComputeScreenCoordinateArrows(
        CoordinateStub &stub, 
        Vector2D<double> &originOut, 
        Vector2D<double> &xHat2DOut, 
        Vector2D<double> &yHat2DOut, 
        Vector2D<double> &zHat2DOut);

    void ShowForwardsDK();
    void ShowInverseDK();

    void ShowControls();

    void ShowDK();

    void ShowEEStatics();
    void ShowJointStatics();
    void ShowBaseTransformStatics();

    void ShowStatics();

public:
    RobotView(Robot *robot);
    RobotView(const RobotView &) = delete;

    void AccumulatePitch(double deltaPitch);
    void AccumulateYaw(double deltaYaw);

    void SnapToTopView();
    void SnapToFrontFiew();
    void SnapToSideView();
    void SnapToIsometricView();

    void IncreasePitch();
    void DecreasePitch();

    void IncreaseYaw();
    void DecreaseYaw();

    void AccumulateScreenOffset(Vector2D<int> mouseDelta);

    void ZoomIn();
    void ZoomOut();

    void ToggleShown(size_t idx);
    void ToggleShowStatics() { this->m_showStatics = !this->m_showStatics; }
    void ShowAll();

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
    void PollChangeEndEffector();

    void PollChangeCamera();
    void PollChangeDKMode();

    void PollToggleShown();

    void PollControlEEStatics();

public:
    RobotController(Robot *robot, RobotView *view);
    RobotController(const RobotController &) = delete;

    void Poll();
};

