#pragma once

#include <vector>
#include <algorithm>

#include "RobotFwd.hpp"

#include "Vector3D.hpp"

#include "Rotation.hpp"
#include "Transform.hpp"
#include "Simulator.hpp"

#include "Jacobian.hpp"

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

    Jacobian m_jacobian;
    bool m_dkForwards = true;


    void ShiftedEndEffector(
        size_t qIter, 
        Vector3D<double> &linearOut, 
        Vector3D<double> &angularOut);

    std::vector<double> GetForwardDKResults();
    std::vector<double> GetInverseDKResults();

    void RecomputeCoordinateStubs();
    void RecomputeJacobian();

public:
    Robot();

    DHParam &GetJoint() { return m_dhParams[this->m_idx]; }
    CoordinateStub &GetCoordinateStub() { return m_coordinateStubs[this->m_idx]; }

    std::vector<CoordinateStub> GetAllStubs() { return this->m_coordinateStubs; }
    std::vector<DHParam> GetAllDHParams() { return this->m_dhParams; }

    std::vector<double> GetDKResults();

    void IncrementIdx();
    void DecrementIdx();
    void SelectIdx(int idx) { this->m_idx = idx; }
    int GetIdx() { return this->m_idx; }

    void Recompute();

    Vector3D<double> GetEndEffectorPosition() { return m_coordinateStubs.back().Position; }
    
    bool IsDKForwards() { return m_dkForwards; }
    void ToggleDKForwards() { m_dkForwards = !m_dkForwards; }

    void AccumulateJointSpeed(double deltaQ) { this->m_manualJointSpeeds[this->m_idx] += deltaQ; }
    void AccumulateEndEffectorVelocity(double deltaE) 
    { this->m_manualEndEffectorVelocity[this->m_idx] += deltaE; }

    std::vector<double> GetEndEffectorVelocity() { return this->m_manualEndEffectorVelocity; }
    std::vector<double> GetManualJointSpeeds() { return this->m_manualJointSpeeds; }
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

public:
    RobotController(Robot *robot, RobotView *view);
    RobotController(const RobotController &) = delete;

    void Poll();
};

