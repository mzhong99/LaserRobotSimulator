#include "Robot.hpp"
#include <cmath> 
#include <SDL.h>

#include <iomanip>

#define ANGULAR_VELOCITY        (3.0)

#define DK_SCREEN_OFFSET        (Vector2D<int>(24, 24))
#define CONTROLS_OFFSET         (Vector2D<int>(32, 560))

RobotView::RobotView(Robot *robot)
{
    m_robot = robot;

    m_shown.resize(m_robot->NumJoints(), true);
    m_showStatics = true;
}

// void RobotView::ComputeScreenCoordinateArrows(
//     CoordinateStub &stub,
//     Vector2D<double> &originOut,
//     Vector2D<double> &xHat2DOut,
//     Vector2D<double> &yHat2DOut,
//     Vector2D<double> &zHat2DOut)
// {
//     Transform stubToBase = stub.GetTransform();
// 
//     Vector3D<double> origin3D = stubToBase.GetColumn(3);
// 
//     Vector3D<double> offsetXHat3D = stubToBase.GetColumn(0);
//     Vector3D<double> offsetYHat3D = stubToBase.GetColumn(1);
//     Vector3D<double> offsetZHat3D = stubToBase.GetColumn(2);
// 
//     Vector3D<double> pointXHat3D = origin3D + offsetXHat3D;
//     Vector3D<double> pointYHat3D = origin3D + offsetYHat3D;
//     Vector3D<double> pointZHat3D = origin3D + offsetZHat3D;
// 
//     originOut = m_camera.WorldToScreen(origin3D);
//     xHat2DOut = m_camera.WorldToScreen(pointXHat3D);
//     yHat2DOut = m_camera.WorldToScreen(pointYHat3D);
//     zHat2DOut = m_camera.WorldToScreen(pointZHat3D);
// }

void RobotView::ShowForwardsDK()
{
    std::vector<double> forwardsResult = this->m_robot->GetDKResults();
    std::vector<double> manualJointSpeeds = this->m_robot->GetManualJointSpeeds();

    Vector3D<double> endEffectorPosition = this->m_robot->GetEndEffectorPosition();
    Vector3D<double> angular;
    angular.x = atan2(endEffectorPosition.z, endEffectorPosition.y);
    angular.y = atan2(endEffectorPosition.x, endEffectorPosition.z);
    angular.z = atan2(endEffectorPosition.y, endEffectorPosition.x);

    std::vector<DHParam> dhparams = this->m_robot->GetAllDHParams();

    Vector2D<int> offset = DK_SCREEN_OFFSET;
    Simulator::Graphics().PrintString(offset.x, offset.y, 
        "Joint List. [Forwards Differential Kinematics]");

    for (size_t i = 0; i < manualJointSpeeds.size(); i++)
    {
        if (i == this->m_robot->GetIdx())
            Simulator::Graphics().SetFGColor(Graphics::COLOR_BLUE);

        Simulator::Graphics().PrintString(offset.x, offset.y + (12 * (i + 1)),
            "    Q%d = %+3.3f    Q%d' = %+3.3f %s    Th: %+.3f  Alp: %+.3f  A: %+.3f  D:%+.3f",
            (int)i + 1, dhparams[i].GetQ(), 
            (int)i + 1, manualJointSpeeds[i],
            dhparams[i].Type == JointType::PRISMATIC ? "m/s  " : "rad/s",
            dhparams[i].Theta, dhparams[i].Alpha, dhparams[i].A, dhparams[i].D);

        if (i == this->m_robot->GetIdx())
        {
            Simulator::Graphics().PrintString(offset.x, offset.y + (12 * ((int)i + 1)), " -> ");
            Simulator::Graphics().SetFGColor(Graphics::COLOR_WHITE);
        }
    }

    Simulator::Graphics().PrintString(offset.x, offset.y + 600, "End Effector Position");

    Simulator::Graphics().PrintString(offset.x, offset.y + 612, 
        "     Linear: <lx=%+3.3f, ly=%+3.3f, lz=%+3.3f> m", 
        endEffectorPosition.x, endEffectorPosition.y, endEffectorPosition.z);

    Simulator::Graphics().PrintString(offset.x, offset.y + 624, 
        "    Angular: <ax=%+3.3f, ay=%+3.3f, az=%+3.3f> rad", 
        angular.x, angular.y, angular.z);

    Simulator::Graphics().PrintString(offset.x, offset.y + 648, "End Effector Velocity");
    Simulator::Graphics().PrintString(offset.x, offset.y + 660, 
        "     Linear: <lx=%+3.3f, ly=%+3.3f, lz=%+3.3f> m/s", 
        forwardsResult[0], forwardsResult[1], forwardsResult[2]);

    Simulator::Graphics().PrintString(offset.x, offset.y + 672, 
        "    Angular: <wx=%+3.3f, wy=%+3.3f, wz=%+3.3f> rad/s", 
        forwardsResult[3], forwardsResult[4], forwardsResult[5]);
}

void RobotView::ShowInverseDK()
{
    Vector2D<int> offset = DK_SCREEN_OFFSET;
    Simulator::Graphics().PrintString(offset.x, offset.y, 
        "Joint List. [Inverse Differential Kinematics]");

    std::vector<double> jointVelocities = this->m_robot->GetDKResults();
    std::vector<DHParam> dhparams = this->m_robot->GetAllDHParams();

    for (size_t i = 0; i < jointVelocities.size(); i++)
    {
        if (i == this->m_robot->GetIdx())
            Simulator::Graphics().SetFGColor(Graphics::COLOR_BLUE);

        Simulator::Graphics().PrintString(offset.x, offset.y + (12 * (i + 1)),
            "    Q%d = %+3.3f    Q%d' = %+3.3f %s %s",
            (int)i + 1, dhparams[i].GetQ(), 
            (int)i + 1, jointVelocities[i], 
            dhparams[i].Type == JointType::PRISMATIC ? "m/s" : "rad/s",
            this->m_robot->IsFreeJoint(i) ? "[Free]" : "");

        if (i == this->m_robot->GetIdx())
        {
            Simulator::Graphics().PrintString(offset.x, offset.y + (12 * ((int)i + 1)), " -> ");
            Simulator::Graphics().SetFGColor(Graphics::COLOR_WHITE);
        }
    }

    std::vector<double> endEffectorVelocity = this->m_robot->GetEndEffectorVelocity();

    Simulator::Graphics().PrintString(offset.x, offset.y + 600, "End Effector Position");

    Vector3D<double> endEffectorPosition = this->m_robot->GetEndEffectorPosition();
    Vector3D<double> angular;
    angular.x = atan2(endEffectorPosition.z, endEffectorPosition.y);
    angular.y = atan2(endEffectorPosition.x, endEffectorPosition.z);
    angular.z = atan2(endEffectorPosition.y, endEffectorPosition.x);

    Simulator::Graphics().PrintString(offset.x, offset.y + 612, 
        "     Linear: <lx=%+3.3f, ly=%+3.3f, lz=%+3.3f> m", 
        endEffectorPosition.x, endEffectorPosition.y, endEffectorPosition.z);

    Simulator::Graphics().PrintString(offset.x, offset.y + 624, 
        "    Angular: <ax=%+3.3f, ay=%+3.3f, az=%+3.3f> rad", 
        angular.x, angular.y, angular.z);

    Simulator::Graphics().PrintString(offset.x, offset.y + 648, "End Effector Velocity");
    Simulator::Graphics().PrintString(offset.x, offset.y + 660, 
        "     Linear: <lx=%+3.3f, ly=%+3.3f, lz=%+3.3f> m/s", 
        endEffectorVelocity[0], endEffectorVelocity[1], endEffectorVelocity[2]);
}

void RobotView::ShowEEStatics()
{
    Transform stubToBase = this->m_robot->EndEffectorTransform();
    Vector3D<double> origin3D = this->m_robot->GetEndEffectorPosition();

    Matrix eeForceMoment = this->m_robot->GetEEForceMoment();

    Vector3D<double> force = Vector3D<double>(
        eeForceMoment.At(0, 0), eeForceMoment.At(1, 0), eeForceMoment.At(2, 0));
    Vector3D<double> moment = Vector3D<double>(
        eeForceMoment.At(3, 0), eeForceMoment.At(4, 0), eeForceMoment.At(5, 0));

    Vector3D<double> forcePos = force + origin3D;
    Vector3D<double> momentPos = moment + origin3D;

    Simulator::Graphics().SetFGColor(Graphics::COLOR_RED);
    this->m_camera.DrawArrow(origin3D, forcePos,
        "F_e=<%.3f, %.3f, %.3f> N", force.x, force.y, force.z);

    this->m_camera.DrawArrow(origin3D, momentPos,
        "M_e=<%.3f, %.3f, %.3f> Nm", moment.x, moment.y, moment.z);
    Simulator::Graphics().SetFGColor(Graphics::COLOR_WHITE);
}

void RobotView::RenderJointStaticReaction(
    CoordinateStub &stub, double reaction, JointType jointType, int frameNumber)
{
    Transform stubToBase = stub.GetTransform();

    Vector3D<double> origin3D = stubToBase.GetColumn(3);
    Vector3D<double> offset3D = stubToBase.GetColumn(2) * reaction;

    Vector3D<double> rxnPoint3D = origin3D + offset3D;

    Simulator::Graphics().SetFGColor(Graphics::COLOR_YELLOW);
    this->m_camera.DrawArrow(origin3D, rxnPoint3D,
        "%s_Q%d=%.3lf%s",
        jointType == JointType::PRISMATIC ? "F" : "M",
        frameNumber,
        reaction,
        jointType == JointType::PRISMATIC ? "N" : "Nm");
    Simulator::Graphics().SetFGColor(Graphics::COLOR_WHITE);
}

void RobotView::ShowJointStatics()
{
    std::vector<CoordinateStub> stubs = this->m_robot->GetAllStubs();
    std::vector<DHParam> dhparams = this->m_robot->GetAllDHParams();
    std::vector<double> reactions = this->m_robot->JointForceMomentReactions();

    CoordinateStub prevStub = CoordinateStub();

    for (size_t i = 0; i < stubs.size(); i++)
    {
        if (this->m_shown[i])
            this->RenderJointStaticReaction(prevStub, reactions[i], dhparams[i].Type, i + 1);

        prevStub = stubs[i];
    }
}

void RobotView::ShowBaseTransformStatics()
{
    Vector3D<double> baseForceEquivalent = this->m_robot->BaseForceEquivalent();
    Vector3D<double> baseMomentEquivalent = this->m_robot->BaseMomentEquivalent();

    Vector3D<double> origin3D = Vector3D<double>(0, 0, 0); /* because it's literally the origin */

    Simulator::Graphics().SetFGColor(Graphics::COLOR_CYAN);

    this->m_camera.DrawArrow(origin3D, baseForceEquivalent,
        "F_eq=<%.3lf, %.3lf, %.3lf> N",
        baseForceEquivalent.x, baseForceEquivalent.y, baseForceEquivalent.z);

    this->m_camera.DrawArrow(origin3D, baseMomentEquivalent,
        "M_eq=<%.3lf, %.3lf, %.3lf> Nm",
        baseMomentEquivalent.x, baseMomentEquivalent.y, baseMomentEquivalent.z);
    Simulator::Graphics().SetFGColor(Graphics::COLOR_WHITE);
}

void RobotView::ShowStatics()
{
    if (this->m_showStatics)
    {
        this->ShowEEStatics();
        this->ShowJointStatics();
        this->ShowBaseTransformStatics();
    }
}

void RobotView::ShowDK()
{
    if (this->m_robot->IsDKForwards())
        this->ShowForwardsDK();
    else
        this->ShowInverseDK();
}

void RobotView::RenderStub(CoordinateStub &stub, bool highlighted, size_t frameNumber)
{
    Transform transform = stub.GetTransform();
    Vector3D<double> origin3D = transform.GetColumn(3);
    Vector3D<double> xHat3D = transform.GetColumn(0) + origin3D;
    Vector3D<double> yHat3D = transform.GetColumn(1) + origin3D;
    Vector3D<double> zHat3D = transform.GetColumn(2) + origin3D;

    Simulator::Graphics().SetFGColor(highlighted ? Graphics::COLOR_BLUE : Graphics::COLOR_WHITE);
    this->m_camera.DrawArrow(origin3D, xHat3D, "x%u", frameNumber);
    this->m_camera.DrawArrow(origin3D, yHat3D, "y%u", frameNumber);

    Simulator::Graphics().SetFGColor(Graphics::COLOR_GREEN);
    this->m_camera.DrawArrow(origin3D, zHat3D, "z%u", frameNumber);
    Simulator::Graphics().SetFGColor(Graphics::COLOR_WHITE);
}

void RobotView::ToggleShown(size_t idx)
{
    if (idx < this->m_shown.size())
        this->m_shown[idx] = !this->m_shown[idx];
}

void RobotView::ShowControls()
{
    Vector2D<int> offset = CONTROLS_OFFSET;
    Simulator::Graphics().PrintString(offset.x, offset.y, "Controls:");

    Simulator::Graphics().PrintString(offset.x, offset.y + 12, 
        "    WASD or Middle Mouse Button: Camera");
    Simulator::Graphics().PrintString(offset.x, offset.y + 24,
        "    ZXCV: Top/Front/Side/Isometric View");

    Simulator::Graphics().PrintString(offset.x, offset.y + 48, 
        "    1-6: Toggle Axis View");
    Simulator::Graphics().PrintString(offset.x, offset.y + 60,
        "    Shift + 1-6: Solo Toggle Axis View");
    Simulator::Graphics().PrintString(offset.x, offset.y + 72,
        "    Backtick (next to the 1 key): Show all Axes)");

    Simulator::Graphics().PrintString(offset.x, offset.y + 96,
        "    Q: Toggle Forwards/Backwards Differential Kinematics");
    Simulator::Graphics().PrintString(offset.x, offset.y + 108,
        "    Right Mouse Button: Pan 2D View");
    Simulator::Graphics().PrintString(offset.x, offset.y + 120,
        "    Left Mouse Button: Adjust selected component");
    Simulator::Graphics().PrintString(offset.x, offset.y + 132,
        "    Shift + Left Mouse Button: Adjust joint velocity");

}

void RobotView::Poll()
{
    CoordinateStub baseFrame;
    this->m_camera.RefreshView();
    this->RenderStub(baseFrame, false, 0);

    std::vector<CoordinateStub> stubs = this->m_robot->GetAllStubs();

    for (size_t i = 0; i < stubs.size(); i++)
        if (this->m_shown[i])
            this->RenderStub(stubs[i], (int)i == this->m_robot->GetIdx(), i + 1);

    this->ShowDK();
    this->ShowStatics();
    // this->ShowControls();
}

void RobotView::ShowSolo(size_t idx)
{
    std::fill(this->m_shown.begin(), this->m_shown.end(), false);

    if (idx < 6)
    {
        this->m_shown[idx] = true;
        if (idx > 0)
            this->m_shown[idx - 1] = true;
    }
}
