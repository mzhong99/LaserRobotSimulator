#include "Robot.hpp"
#include <cmath> 
#include <SDL.h>

#include <iomanip>

#define DEFAULT_ZOOM_FACTOR     (72.0)
#define ZOOM_VELOCITY           (15.0)

#define ANGULAR_VELOCITY        (3.0)

#define DK_SCREEN_OFFSET        (Vector2D<int>(24, 24))
#define CONTROLS_OFFSET         (Vector2D<int>(32, 560))

RobotView::RobotView(Robot *robot)
{
    this->m_robot = robot;
    this->m_zoomFactor = DEFAULT_ZOOM_FACTOR;

    this->m_yaw = -1.0 * M_PI / 3.0;
    this->m_pitch = -1.0 * M_PI / 6.0;

    this->m_shown.resize(this->m_robot->GetAllStubs().size(), true);
    this->m_showStatics = true;
}

void RobotView::IncreasePitch()
{
    double deltaTimeSeconds = Simulator::Application().DeltaTimeMS() / 1000.0;
    double increment = ANGULAR_VELOCITY * deltaTimeSeconds;

    if (this->m_pitch + increment < M_PI)
        this->m_pitch += increment;
}

void RobotView::DecreasePitch()
{
    double deltaTimeSeconds = Simulator::Application().DeltaTimeMS() / 1000.0;
    double increment = ANGULAR_VELOCITY * deltaTimeSeconds;

    if (this->m_pitch - increment > 0)
        this->m_pitch -= increment;
}

void RobotView::IncreaseYaw()
{
    double deltaTimeSeconds = Simulator::Application().DeltaTimeMS() / 1000.0;
    double increment = ANGULAR_VELOCITY * deltaTimeSeconds;

    this->m_yaw += increment;
}

void RobotView::DecreaseYaw()
{
    double deltaTimeSeconds = Simulator::Application().DeltaTimeMS() / 1000.0;
    double increment = ANGULAR_VELOCITY * deltaTimeSeconds;

    this->m_yaw -= increment;
}

void RobotView::ZoomIn()
{
    double deltaTimeSeconds = Simulator::Application().DeltaTimeMS() / 1000.0;
    double increment = ZOOM_VELOCITY * deltaTimeSeconds;

    this->m_zoomFactor += increment;
}

void RobotView::ZoomOut()
{
    double deltaTimeSeconds = Simulator::Application().DeltaTimeMS() / 1000.0;
    double increment = ZOOM_VELOCITY * deltaTimeSeconds;

    this->m_zoomFactor -= increment;
}

void RobotView::AccumulatePitch(double deltaPitch)
{
    this->m_pitch += deltaPitch;

    if (this->m_pitch > M_PI)
        this->m_pitch = M_PI;

    if (this->m_pitch < 0)
        this->m_pitch = 0;
}

void RobotView::AccumulateYaw(double deltaYaw)
{
    this->m_yaw += deltaYaw;
}

Rotation RobotView::GetSphericalRotation()
{
    Rotation aboutX = Rotation::CreateAboutX(m_pitch);
    Rotation aboutY = Rotation::CreateAboutY(m_yaw);

    return Rotation::Multiply(aboutX, aboutY);
}

Vector2D<double> RobotView::WorldToScreen(Vector3D<double> worldVector)
{
    if (worldVector.z < -0.9)
        return Vector2D<double>(1e6, 1e6);
    double zw = fabs(worldVector.z);

    double W = Simulator::Graphics().Width();
    double H = Simulator::Graphics().Height();
    double A = W / H;

    double xw = worldVector.x;
    double yw = worldVector.y;

    double thetaFOV = M_PI * (75.0 / 180.0);
    double tangent = tan(thetaFOV / 2.0);

    double xv = xw / zw;
    double yv = yw / zw;

    double xi = 0.5 * (W * ((xv / (A * tangent)) + 1.0) - 1.0);
    double yi = 0.5 * (H * (1.0 - (yv / tangent)) - 1.0);

    return Vector2D<double>(xi, yi);
}

void RobotView::ComputeScreenCoordinateArrows(
    CoordinateStub &stub,
    Vector2D<double> &originOut,
    Vector2D<double> &xHat2DOut,
    Vector2D<double> &yHat2DOut,
    Vector2D<double> &zHat2DOut)
{
    Rotation baseToCameraRotation = this->GetSphericalRotation();

    Vector2D<double> xyOffset = m_screenOffset2D * 0.01;
    xyOffset.x *= -1.0;

    // Transform baseToCamera = Transform(
    //     baseToCameraRotation,
    //     Vector3D<double>(xyOffset.x, xyOffset.y, DEFAULT_ZOOM_FACTOR + m_zoomFactor));

    Transform baseToCamera = Transform(baseToCameraRotation, Vector3D<double>(0));

    Transform stubToBase = stub.GetTransform();
    Transform stubToCamera = Transform::Multiply(baseToCamera, stubToBase);

    Vector3D<double> origin3D = stubToCamera.GetColumn(3);

    Vector3D<double> offsetXHat3D = stubToCamera.GetColumn(0);
    Vector3D<double> offsetYHat3D = stubToCamera.GetColumn(1);
    Vector3D<double> offsetZHat3D = stubToCamera.GetColumn(2);

    Vector3D<double> pointXHat3D = origin3D + offsetXHat3D;
    Vector3D<double> pointYHat3D = origin3D + offsetYHat3D;
    Vector3D<double> pointZHat3D = origin3D + offsetZHat3D;

    Vector2D<double> origin2D = Vector2D<double>(origin3D.x, origin3D.z);

    xHat2DOut = Vector2D<double>(pointXHat3D.x, pointXHat3D.z);
    yHat2DOut = Vector2D<double>(pointYHat3D.x, pointYHat3D.z);
    zHat2DOut = Vector2D<double>(pointZHat3D.x, pointZHat3D.z);

    originOut = Simulator::Graphics().LogicToScreen2D(origin2D * this->m_zoomFactor);
    xHat2DOut = Simulator::Graphics().LogicToScreen2D(xHat2DOut * this->m_zoomFactor);
    yHat2DOut = Simulator::Graphics().LogicToScreen2D(yHat2DOut * this->m_zoomFactor);
    zHat2DOut = Simulator::Graphics().LogicToScreen2D(zHat2DOut * this->m_zoomFactor);

    originOut -= this->m_screenOffset2D;
    xHat2DOut -= this->m_screenOffset2D;
    yHat2DOut -= this->m_screenOffset2D;
    zHat2DOut -= this->m_screenOffset2D;
}

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
    Rotation baseToCameraRotation = this->GetSphericalRotation();

    Vector2D<double> xyOffset = m_screenOffset2D * 0.01;
    xyOffset.x *= -1.0;

    Transform stubToBase = this->m_robot->EndEffectorTransform();
    Transform baseToCamera = Transform(baseToCameraRotation, 0);
    Transform stubToCamera = Transform::Multiply(baseToCamera, stubToBase);

    Vector3D<double> origin3D = stubToCamera.GetColumn(3);

    Matrix eeForceMoment = this->m_robot->GetEEForceMoment();

    Vector3D<double> force = Vector3D<double>(
        eeForceMoment.At(0, 0), eeForceMoment.At(1, 0), eeForceMoment.At(2, 0));

    Vector3D<double> moment = Vector3D<double>(
        eeForceMoment.At(3, 0), eeForceMoment.At(4, 0), eeForceMoment.At(5, 0));

    Vector3D<double> forceInCamera = baseToCamera.TransformPoint(force);
    Vector3D<double> momentInCamera = baseToCamera.TransformPoint(moment);

    Vector3D<double> forcePos = forceInCamera + origin3D;
    Vector3D<double> momentPos = momentInCamera + origin3D;
    
    Vector2D<double> origin2D = Vector2D<double>(origin3D.x, origin3D.z);
    Vector2D<double> force2D = Vector2D<double>(forcePos.x, forcePos.z);
    Vector2D<double> moment2D = Vector2D<double>(momentPos.x, momentPos.z);

    Vector2D<double> screenOrigin = Simulator::Graphics().LogicToScreen2D(
        origin2D * this->m_zoomFactor);

    Vector2D<double> screenForcePos = Simulator::Graphics().LogicToScreen2D(
        force2D * this->m_zoomFactor);

    Vector2D<double> screenMomentPos = Simulator::Graphics().LogicToScreen2D(
        moment2D * this->m_zoomFactor);

    screenOrigin -= this->m_screenOffset2D;
    screenForcePos -= this->m_screenOffset2D;
    screenMomentPos -= this->m_screenOffset2D;

    Simulator::Graphics().SetFGColor(Graphics::COLOR_RED);
    Simulator::Graphics().DrawArrow(screenOrigin, screenForcePos,
        "F_e=<%.3f, %.3f, %.3f> N", force.x, force.y, force.z);

    Simulator::Graphics().DrawArrow(screenOrigin, screenMomentPos,
        "M_e=<%.3f, %.3f, %.3f> Nm", moment.x, moment.y, moment.z);
    Simulator::Graphics().SetFGColor(Graphics::COLOR_WHITE);
}

void RobotView::RenderJointStaticReaction(
    CoordinateStub &stub, double reaction, JointType jointType, int frameNumber)
{
    Rotation baseToCameraRotation = this->GetSphericalRotation();

    Vector2D<double> xyOffset = m_screenOffset2D * 0.01;
    xyOffset.x *= -1.0;

    Transform stubToBase = stub.GetTransform();
    Transform baseToCamera = Transform(baseToCameraRotation, 0);
    Transform stubToCamera = Transform::Multiply(baseToCamera, stubToBase);

    Vector3D<double> origin3D = stubToCamera.GetColumn(3);
    Vector3D<double> offset3D = stubToCamera.GetColumn(2) * reaction;

    Vector3D<double> rxnPoint3D = origin3D + offset3D;

    Vector2D<double> origin2D = Vector2D<double>(origin3D.x, origin3D.z);
    Vector2D<double> rxnPoint2D = Vector2D<double>(rxnPoint3D.x, rxnPoint3D.z);

    Vector2D<double> scOrigin2D = Simulator::Graphics().LogicToScreen2D(
        origin2D * this->m_zoomFactor);

    Vector2D<double> scRxnPoint2D = Simulator::Graphics().LogicToScreen2D(
        rxnPoint2D * this->m_zoomFactor);

    scOrigin2D -= this->m_screenOffset2D;
    scRxnPoint2D -= this->m_screenOffset2D;

    Simulator::Graphics().SetFGColor(Graphics::COLOR_YELLOW);
    Simulator::Graphics().DrawArrow(scOrigin2D, scRxnPoint2D,
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

    Rotation baseToCameraRotation = this->GetSphericalRotation();

    Vector2D<double> xyOffset = m_screenOffset2D * 0.01;
    xyOffset.x *= -1.0;

    Transform baseToCamera = Transform(baseToCameraRotation, Vector3D<double>(0));

    Vector3D<double> origin3D = baseToCamera.GetColumn(3);
    Vector2D<double> origin2D = Vector2D<double>(origin3D.x, origin3D.z);

    Vector3D<double> cameraForce = baseToCamera.TransformPoint(baseForceEquivalent);
    Vector3D<double> cameraMoment = baseToCamera.TransformPoint(baseMomentEquivalent);

    cameraForce -= origin3D;
    cameraMoment -= origin3D;

    Vector2D<double> force2D = Vector2D<double>(cameraForce.x, cameraForce.z);
    Vector2D<double> moment2D = Vector2D<double>(cameraMoment.x, cameraMoment.z);

    Vector2D<double> screenOrigin = Simulator::Graphics().LogicToScreen2D(
        origin2D * this->m_zoomFactor);

    Vector2D<double> screenForce = Simulator::Graphics().LogicToScreen2D(
        force2D * this->m_zoomFactor);

    Vector2D<double> screenMoment = Simulator::Graphics().LogicToScreen2D(
        moment2D * this->m_zoomFactor);

    screenOrigin -= this->m_screenOffset2D;
    screenForce -= this->m_screenOffset2D;
    screenMoment -= this->m_screenOffset2D;


    Simulator::Graphics().SetFGColor(Graphics::COLOR_CYAN);
    Simulator::Graphics().DrawArrow(screenOrigin, screenForce,
        "F_eq=<%.3lf, %.3lf, %.3lf> N",
        baseForceEquivalent.x, baseForceEquivalent.y, baseForceEquivalent.z);

    Simulator::Graphics().DrawArrow(screenOrigin, screenMoment,
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
    Vector2D<double> origin2D, pointXHat2D, pointYHat2D, pointZHat2D;
    this->ComputeScreenCoordinateArrows(stub, origin2D, pointXHat2D, pointYHat2D, pointZHat2D);

    Simulator::Graphics().SetFGColor(highlighted ? Graphics::COLOR_BLUE : Graphics::COLOR_WHITE);
    Simulator::Graphics().DrawArrow(origin2D, pointXHat2D, "x%u", frameNumber);
    Simulator::Graphics().DrawArrow(origin2D, pointYHat2D, "y%u", frameNumber);

    Simulator::Graphics().SetFGColor(Graphics::COLOR_GREEN);
    Simulator::Graphics().DrawArrow(origin2D, pointZHat2D, "z%u", frameNumber);
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
    this->RenderStub(baseFrame, false, 0);

    std::vector<CoordinateStub> stubs = this->m_robot->GetAllStubs();

    for (size_t i = 0; i < stubs.size(); i++)
        if (this->m_shown[i])
            this->RenderStub(stubs[i], (int)i == this->m_robot->GetIdx(), i + 1);

    this->ShowDK();
    this->ShowStatics();
    // this->ShowControls();
}

void RobotView::AccumulateScreenOffset(Vector2D<int> mouseDelta)
{
    Vector2D<double> delta = Vector2D<double>(mouseDelta.x, mouseDelta.y);
    this->m_screenOffset2D -= delta;
}

void RobotView::SnapToTopView()
{
    this->m_yaw = 0;
    this->m_pitch = 0;
}

void RobotView::SnapToFrontFiew()
{
    this->m_yaw = 0;
    this->m_pitch = M_PI / 2.0;
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
