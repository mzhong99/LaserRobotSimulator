#include "Robot.hpp"
#include <cmath> 
#include <SDL.h>

#include <iomanip>

#define INCREMENT_VELOCITY      (3.0)
#define DEFAULT_DISTANCE        (0.0)

#define ANGULAR_VELOCITY        (3.0)

// #define DEFAULT_ZOOM_FACTOR     (3.0)
// #define ZOOM_VELOCITY           (3.0)

#define DEFAULT_ZOOM_FACTOR     (72.0)
#define ZOOM_VELOCITY           (15.0)

#define LONGITUDE_SENSITIVITY   (0.0025)
#define LATITUDE_SENSITIVITY    (0.005)

#define DIFFERENTIAL_DELTA      (0.00001)
#define DK_SCREEN_OFFSET        (Vector2D<int>(32, 64))

#define CONTROLS_OFFSET         (Vector2D<int>(32, 560))

DHParam DHParam::CreateRevolute(double alpha, double a, double d)
{
    DHParam param;

    param.Alpha = alpha;
    param.A = a;
    param.D = d;
    param.Theta = 1.0;

    param.Type = JointType::REVOLUTE;

    return param;
}

DHParam DHParam::CreatePrismatic(double theta, double alpha, double a)
{
    DHParam param;

    param.Theta = theta;
    param.Alpha = alpha;
    param.A = a;
    param.D = 3.0;

    param.Type = JointType::PRISMATIC;

    return param;
}

double DHParam::GetQ() 
{
    switch (this->Type)
    {
        case JointType::PRISMATIC:
            return this->D;
        case JointType::REVOLUTE:
            return this->Theta;
    }

    return 0;
}

void DHParam::SetQ(double val)
{
    switch (this->Type)
    {
        case JointType::PRISMATIC:
            this->D = val;
            break;

        case JointType::REVOLUTE:
            this->Theta = val;
            break;

        default:
            break;
    }
}

/**************************************************************************************************/
/* Model ---------------------------------------------------------------------------------------- */
/**************************************************************************************************/

Robot::Robot()
{
    m_dhParams.push_back(DHParam::CreatePrismatic(M_PI / 2.0, M_PI / 2.0, 0));
    m_dhParams.push_back(DHParam::CreatePrismatic(M_PI / 2.0, M_PI / 2.0, 0));
    m_dhParams.push_back(DHParam::CreateRevolute(0, 0, 2.0));
    m_dhParams.push_back(DHParam::CreateRevolute(M_PI / 2.0, 0, 0));
    m_dhParams.push_back(DHParam::CreateRevolute(M_PI / 2.0, 0, 0));
    m_dhParams.push_back(DHParam::CreatePrismatic(0, 0, 0));

    m_coordinateStubs.resize(m_dhParams.size());
    m_manualJointSpeeds.resize(m_dhParams.size());
    m_manualEndEffectorVelocity.resize(m_dhParams.size());
}

void Robot::RecomputeCoordinateStubs()
{
    Transform cumulative = Transform();

    for (size_t i = 0; i < m_dhParams.size(); i++)
    {
        DHParam dhparam = m_dhParams[i];
        Transform transform = Transform(dhparam.Theta, dhparam.Alpha, dhparam.A, dhparam.D);
        
        /* Perform a right-multiplication since cumulative contains all but the latest */
        cumulative = Transform::Multiply(cumulative, transform);

        Vector3D<double> position = cumulative.GetDisplacement();
        Rotation rotation = cumulative.GetRotation();

        CoordinateStub stub = CoordinateStub(position, rotation);
        m_coordinateStubs[i] = stub;
    }
}

void Robot::ShiftedEndEffector(
    size_t qIter,
    Vector3D<double> &linearOut,
    Vector3D<double> &angularOut)
{
    Transform cumulative = Transform();

    for (size_t i = 0; i < m_dhParams.size(); i++)
    {
        DHParam dhparam = m_dhParams[i];

        if (i == qIter)
        {
            switch (dhparam.Type)
            {
                case JointType::REVOLUTE  : dhparam.Theta += DIFFERENTIAL_DELTA; break; 
                case JointType::PRISMATIC : dhparam.D     += DIFFERENTIAL_DELTA; break;
                default: break;
            }
        }

        Transform transform = Transform(dhparam.Theta, dhparam.Alpha, dhparam.A, dhparam.D);
        if (i == qIter && qIter == 5)
        {
            if (Simulator::Input().KeyTapped(SDLK_2) && Simulator::Input().KeyPressed(SDLK_LCTRL))
            {
                std::cout << std::fixed << std::setprecision(8);
                std::cout << "Last Transform:" << std::endl;
                for (int row = 0; row < 4; row++)
                {
                    std::cout << "[ ";
                    for (int col = 0; col < 4; col++)
                        std::cout << transform.Get(row, col) << " ";
                    std::cout << "]" << std::endl;
                }

                Transform transform = Transform(dhparam.Theta, dhparam.Alpha, dhparam.A, dhparam.D);
            }
        }

        cumulative = Transform::Multiply(cumulative, transform);
    }

    linearOut = Vector3D<double>(cumulative.GetDisplacement());
    angularOut = Vector3D<double>();

    angularOut.x = atan2(linearOut.z, linearOut.y);
    angularOut.y = atan2(linearOut.x, linearOut.z);
    angularOut.z = atan2(linearOut.y, linearOut.x);
}

void Robot::RecomputeJacobian()
{
    for (size_t qIter = 0; qIter < this->m_dhParams.size(); qIter++)
    {
        Vector3D<double> linearPlusH, angularPlusH;
        this->ShiftedEndEffector(qIter, linearPlusH, angularPlusH);

        Vector3D<double> linear = this->m_coordinateStubs.back().Position;

        Vector3D<double> angular;
        angular.x = atan2(linear.z, linear.y);
        angular.y = atan2(linear.x, linear.z);
        angular.z = atan2(linear.y, linear.x);

        Vector3D<double> linearDelta = (linearPlusH - linear) * (1.0 / DIFFERENTIAL_DELTA);
        Vector3D<double> angularDelta = (angularPlusH - angular) * (1.0 / DIFFERENTIAL_DELTA);

        this->m_jacobian.Set(0, qIter, linearDelta.x);
        this->m_jacobian.Set(1, qIter, linearDelta.y);
        this->m_jacobian.Set(2, qIter, linearDelta.z);

        this->m_jacobian.Set(3, qIter, angularDelta.x);
        this->m_jacobian.Set(4, qIter, angularDelta.y);
        this->m_jacobian.Set(5, qIter, angularDelta.z);
    }

    if (Simulator::Input().KeyTapped(SDLK_1) && Simulator::Input().KeyPressed(SDLK_LCTRL))
    {
        std::cout << "Jacobian:" << std::endl;
        for (size_t row = 0; row < 6; row++)
        {
            std::cout << "[ ";
            for (size_t col = 0; col < 6; col++)
                std::cout << this->m_jacobian.Get(row, col) << " ";
            std::cout << "]" << std::endl;
        }

        std::cout << "Jacobian (Inverted):" << std::endl;
        Jacobian inverted = this->m_jacobian.Inverted();
        for (size_t row = 0; row < 6; row++)
        {
            std::cout << "[ ";
            for (size_t col = 0; col < 6; col++)
                std::cout << inverted.Get(row, col) << " ";
            std::cout << "]" << std::endl;
        }

        std::cout << "Jacobian Multiplied" << std::endl;
        Jacobian multiplied = Jacobian::Multiply(m_jacobian, inverted);

        for (size_t row = 0; row < 6; row++)
        {
            std::cout << "[ ";
            for (size_t col = 0; col < 6; col++)
                std::cout << multiplied.Get(row, col) << " ";
            std::cout << "]" << std::endl;
        }
    }
}

std::vector<double> Robot::GetForwardDKResults()
{
    Vector3D<double> linear, angular;
    this->m_jacobian.ForwardMultiply(m_manualJointSpeeds, linear, angular);

    std::vector<double> result = { linear.x, linear.y, linear.z, angular.x, angular.y, angular.z };
    return result;
}

std::vector<double> Robot::GetInverseDKResults()
{
    Jacobian inverted = this->m_jacobian.Inverted();

    Vector3D<double> linear = Vector3D<double>(
        this->m_manualEndEffectorVelocity[0],
        this->m_manualEndEffectorVelocity[1],
        this->m_manualEndEffectorVelocity[2]);

    Vector3D<double> angular = Vector3D<double>(
        this->m_manualEndEffectorVelocity[3],
        this->m_manualEndEffectorVelocity[4],
        this->m_manualEndEffectorVelocity[5]);

    std::vector<double> result = inverted.InverseMultiply(linear, angular);
    return result;
}

std::vector<double> Robot::GetDKResults()
{
    std::vector<double> result;

    if (m_dkForwards)
        result = this->GetForwardDKResults();
    else
        result = this->GetInverseDKResults();

    return result;
}

void Robot::Recompute()
{
    this->RecomputeCoordinateStubs();
    this->RecomputeJacobian();
}

void Robot::IncrementIdx()
{
    if (this->m_idx + (size_t)1 < this->m_dhParams.size())
        this->m_idx++;
}

void Robot::DecrementIdx()
{
    if (this->m_idx > 0)
        this->m_idx--;
}

/**************************************************************************************************/
/* View ----------------------------------------------------------------------------------------- */
/**************************************************************************************************/
RobotView::RobotView(Robot *robot)
{
    this->m_robot = robot;
    this->m_zoomFactor = DEFAULT_ZOOM_FACTOR;

    this->m_yaw = -1.0 * M_PI / 3.0;
    this->m_pitch = -1.0 * M_PI / 6.0;

    this->m_shown.resize(this->m_robot->GetAllStubs().size(), true);
}

void RobotView::IncreasePitch()
{
    double deltaTimeSeconds = Simulator::Application().DeltaTimeMS() / 1000.0;
    double increment = ANGULAR_VELOCITY * deltaTimeSeconds;

    if (this->m_pitch + increment < M_PI / 2.0)
        this->m_pitch += increment;
}

void RobotView::DecreasePitch()
{
    double deltaTimeSeconds = Simulator::Application().DeltaTimeMS() / 1000.0;
    double increment = ANGULAR_VELOCITY * deltaTimeSeconds;

    if (this->m_pitch - increment > -1.0 * M_PI / 2.0)
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

    if (this->m_pitch > M_PI / 2.0)
        this->m_pitch = M_PI / 2.0;

    if (this->m_pitch < -1.0 * M_PI / 2.0)
        this->m_pitch = -1.0 * M_PI / 2.0;
}

void RobotView::AccumulateYaw(double deltaYaw)
{
    this->m_yaw += deltaYaw;
}

Rotation RobotView::GetSphericalRotation()
{
    // double uxZHat = cos(m_pitch) * sin(m_yaw);
    // double uyZHat = cos(m_pitch) * cos(m_yaw);
    // double uzZHat = sin(m_pitch);

    // Vector3D<double> zHat = Vector3D<double>(uxZHat, uyZHat, uzZHat);

    // Vector2D<double> urXHat = Vector2D<double>(zHat.x, zHat.y);
    // urXHat.Rotate(90);

    // Vector3D<double> xHat = Vector3D<double>(urXHat.x, urXHat.y, 0).Normalized();
    // Vector3D<double> yHat = Vector3D<double>::Cross(zHat, xHat);

    // Rotation rotation;

    // rotation.Set(0, 0, xHat.x);
    // rotation.Set(1, 0, xHat.y);
    // rotation.Set(2, 0, xHat.z);

    // rotation.Set(0, 1, yHat.x);
    // rotation.Set(1, 1, yHat.y);
    // rotation.Set(2, 1, yHat.z);

    // rotation.Set(0, 2, zHat.x);
    // rotation.Set(1, 2, zHat.y);
    // rotation.Set(2, 2, zHat.z);

    // return rotation;

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

    // originOut = this->WorldToScreen(origin3D);
    // xHat2DOut = this->WorldToScreen(pointXHat3D);
    // yHat2DOut = this->WorldToScreen(pointYHat3D);
    // zHat2DOut = this->WorldToScreen(pointZHat3D);
}

void RobotView::ShowForwardsDK()
{
    std::vector<double> forwardsResult = this->m_robot->GetDKResults();
    std::vector<double> manualJointSpeeds = this->m_robot->GetManualJointSpeeds();

    Vector3D<double> endEffectorPosition = this->m_robot->GetEndEffectorPosition();
    Vector3D<double> endEffectorAngular;
    endEffectorAngular.x = atan2(endEffectorPosition.z, endEffectorPosition.y);
    endEffectorAngular.y = atan2(endEffectorPosition.x, endEffectorPosition.z);
    endEffectorAngular.z = atan2(endEffectorPosition.y, endEffectorPosition.x);

    std::vector<DHParam> dhparams = this->m_robot->GetAllDHParams();

    Vector2D<int> offset = DK_SCREEN_OFFSET;
    Simulator::Graphics().PrintString(offset.x, offset.y, 
        "Joint List. [Forwards Differential Kinematics]");

    for (size_t i = 0; i < forwardsResult.size(); i++)
    {
        Simulator::Graphics().PrintString(offset.x, offset.y + (12 * (i + 1)),
            "    Q%u = %+3.3f    Q%u' = %+3.3f",
            i + 1, dhparams[i].GetQ(), i + 1, manualJointSpeeds[i]);

        if (i == this->m_robot->GetIdx())
            Simulator::Graphics().PrintString(offset.x, offset.y + (12 * (i + 1)), " -> ");
    }

    Simulator::Graphics().PrintString(offset.x, offset.y + 100, "End Effector Position");

    Simulator::Graphics().PrintString(offset.x, offset.y + 112, 
        "     Linear: <lx=%+3.3f, ly=%+3.3f, lz=%+3.3f> m", 
        endEffectorPosition.x, endEffectorPosition.y, endEffectorPosition.z);

    Simulator::Graphics().PrintString(offset.x, offset.y + 124, 
        "    Angular: <wx=%+3.3f, wy=%+3.3f, wz=%+3.3f> rad",
        endEffectorAngular.x, endEffectorAngular.y, endEffectorAngular.z);

    Simulator::Graphics().PrintString(offset.x, offset.y + 148, "End Effector Velocity");
    Simulator::Graphics().PrintString(offset.x, offset.y + 160, 
        "     Linear: <lx=%+3.3f, ly=%+3.3f, lz=%+3.3f> m/s", 
        forwardsResult[0], forwardsResult[1], forwardsResult[2]);

    Simulator::Graphics().PrintString(offset.x, offset.y + 172, 
        "    Angular: <wx=%+3.3f, wy=%+3.3f, wz=%+3.3f> rad/s",
        forwardsResult[3], forwardsResult[4], forwardsResult[5]);
}

void RobotView::ShowInverseDK()
{
    Vector2D<int> offset = DK_SCREEN_OFFSET;
    Simulator::Graphics().PrintString(offset.x, offset.y, 
        "Joint List. [Inverse Differential Kinematics]");

    std::vector<double> jointVelocities = this->m_robot->GetDKResults();
    for (size_t i = 0; i < jointVelocities.size(); i++)
    {
        Simulator::Graphics().PrintString(offset.x, offset.y + (12 * (i + 1)),
            "    Q%u' = %+3.3f", i + 1, jointVelocities[i]);
    }

    std::vector<double> endEffectorVelocity = this->m_robot->GetEndEffectorVelocity();

    Simulator::Graphics().PrintString(offset.x, offset.y + 148, "End Effector Velocity");
    Simulator::Graphics().PrintString(offset.x, offset.y + 160, 
        "     Linear: <lx=%+3.3f, ly=%+3.3f, lz=%+3.3f> m/s", 
        endEffectorVelocity[0], endEffectorVelocity[1], endEffectorVelocity[2]);

    Simulator::Graphics().PrintString(offset.x, offset.y + 172, 
        "    Angular: <wx=%+3.3f, wy=%+3.3f, wz=%+3.3f> rad/s",
        endEffectorVelocity[3], endEffectorVelocity[4], endEffectorVelocity[5]);
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
    this->ShowControls();
}

void RobotView::AccumulateScreenOffset(Vector2D<int> mouseDelta)
{
    Vector2D<double> delta = Vector2D<double>(mouseDelta.x, mouseDelta.y);
    this->m_screenOffset2D -= delta;
}

void RobotView::SnapToTopView()
{
    this->m_yaw = 0;
    this->m_pitch = M_PI / 2.0;
}

void RobotView::SnapToFrontFiew()
{
    this->m_yaw = 0;
    this->m_pitch = 0;
}

void RobotView::SnapToSideView()
{
    this->m_yaw = M_PI / 2.0;
    this->m_pitch = 0;
}

void RobotView::SnapToIsometricView()
{
    this->m_yaw = -3.0 * M_PI / 4.0;
    this->m_pitch = M_PI / 4.0;
}

void RobotView::ShowAll()
{
    std::fill(this->m_shown.begin(), this->m_shown.end(), true);
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

/**************************************************************************************************/
/* Controller ----------------------------------------------------------------------------------- */
/**************************************************************************************************/
RobotController::RobotController(Robot *robot, RobotView *view)
{
    this->m_robot = robot;
    this->m_view = view;
}

void RobotController::PollChangeIdx()
{
    if (Simulator::Input().KeyTapped(SDLK_LEFT))
        this->m_robot->DecrementIdx();

    if (Simulator::Input().KeyTapped(SDLK_RIGHT))
        this->m_robot->IncrementIdx();

    if (Simulator::Input().KeyPressed(SDLK_LSHIFT))
    {
        if (Simulator::Input().KeyTapped(SDLK_1))
            this->m_robot->SelectIdx(0);

        if (Simulator::Input().KeyTapped(SDLK_2))
            this->m_robot->SelectIdx(1);

        if (Simulator::Input().KeyTapped(SDLK_3))
            this->m_robot->SelectIdx(2);

        if (Simulator::Input().KeyTapped(SDLK_4))
            this->m_robot->SelectIdx(3);

        if (Simulator::Input().KeyTapped(SDLK_5))
            this->m_robot->SelectIdx(4);

        if (Simulator::Input().KeyTapped(SDLK_6))
            this->m_robot->SelectIdx(5);
    }
}

void RobotController::PollChangeQ()
{
    double scaler = 0.0;
    if (Simulator::Input().KeyPressed(SDLK_UP))
        scaler += 1.0;

    if (Simulator::Input().KeyPressed(SDLK_DOWN))
        scaler -= 1.0;

    double deltaTimeSeconds = Simulator::Application().DeltaTimeMS() / 1000.0;
    double increment = scaler * INCREMENT_VELOCITY * deltaTimeSeconds;
    double oldQ = this->m_robot->GetJoint().GetQ();

    this->m_robot->GetJoint().SetQ(oldQ + increment);

    if (Simulator::Input().KeyPressed(SDLK_LSHIFT))
    {
        if (Simulator::Input().MousePressed(SDL_BUTTON_LEFT))
        {
            Vector2D<int> motion = Simulator::Input().MouseMotion();
            this->m_robot->AccumulateJointSpeed(motion.y * -0.005);
        }
    }
    else
    {
        if (Simulator::Input().MousePressed(SDL_BUTTON_LEFT))
        {
            Vector2D<int> motion = Simulator::Input().MouseMotion();
            this->m_robot->GetJoint().SetQ(oldQ + (motion.y * -0.005));
        }
    }
}

void RobotController::PollChangeEndEffector()
{
    if (Simulator::Input().MousePressed(SDL_BUTTON_LEFT))
    {
        Vector2D<int> motion = Simulator::Input().MouseMotion();
        this->m_robot->AccumulateEndEffectorVelocity(motion.y * -0.005);
    }
}

void RobotController::PollChangeCamera()
{
    if (Simulator::Input().KeyPressed(SDLK_w))
        this->m_view->IncreasePitch();

    if (Simulator::Input().KeyPressed(SDLK_s))
        this->m_view->DecreasePitch();

    if (Simulator::Input().KeyPressed(SDLK_a))
        this->m_view->IncreaseYaw();

    if (Simulator::Input().KeyPressed(SDLK_d))
        this->m_view->DecreaseYaw();

    if (Simulator::Input().KeyPressed(SDLK_z))
        this->m_view->SnapToTopView();

    if (Simulator::Input().KeyPressed(SDLK_x))
        this->m_view->SnapToFrontFiew();

    if (Simulator::Input().KeyPressed(SDLK_c))
        this->m_view->SnapToSideView();

    if (Simulator::Input().KeyPressed(SDLK_v))
        this->m_view->SnapToIsometricView();

    if (Simulator::Input().KeyPressed(SDLK_EQUALS))
        this->m_view->ZoomIn();

    if (Simulator::Input().KeyPressed(SDLK_MINUS))
        this->m_view->ZoomOut();

    if (Simulator::Input().MousePressed(SDL_BUTTON_RIGHT))
    {
        Vector2D<int> motion = Simulator::Input().MouseMotion();
        this->m_view->AccumulateScreenOffset(motion);
    }

    if (Simulator::Input().MousePressed(SDL_BUTTON_MIDDLE))
    {
        Vector2D<int> motion = Simulator::Input().MouseMotion();
        this->m_view->AccumulatePitch(-1.0 * motion.y * LATITUDE_SENSITIVITY);
        this->m_view->AccumulateYaw(motion.x * LONGITUDE_SENSITIVITY);
    }
}

void RobotController::PollChangeDKMode()
{
    if (Simulator::Input().KeyTapped(SDLK_q))
        this->m_robot->ToggleDKForwards();
}

void RobotController::PollToggleShown()
{
    if (this->m_robot->IsDKForwards())
    {
        if (Simulator::Input().KeyPressed(SDLK_LSHIFT))
        {
            if (Simulator::Input().KeyTapped(SDLK_1))
                this->m_view->ShowSolo(0);

            if (Simulator::Input().KeyTapped(SDLK_2))
                this->m_view->ShowSolo(1);

            if (Simulator::Input().KeyTapped(SDLK_3))
                this->m_view->ShowSolo(2);

            if (Simulator::Input().KeyTapped(SDLK_4))
                this->m_view->ShowSolo(3);

            if (Simulator::Input().KeyTapped(SDLK_5))
                this->m_view->ShowSolo(4);

            if (Simulator::Input().KeyTapped(SDLK_6))
                this->m_view->ShowSolo(5);
        }
        else
        {
            if (Simulator::Input().KeyTapped(SDLK_1))
                this->m_view->ToggleShown(0);

            if (Simulator::Input().KeyTapped(SDLK_2))
                this->m_view->ToggleShown(1);

            if (Simulator::Input().KeyTapped(SDLK_3))
                this->m_view->ToggleShown(2);

            if (Simulator::Input().KeyTapped(SDLK_4))
                this->m_view->ToggleShown(3);

            if (Simulator::Input().KeyTapped(SDLK_5))
                this->m_view->ToggleShown(4);

            if (Simulator::Input().KeyTapped(SDLK_6))
                this->m_view->ToggleShown(5);
        }
    }

    if (Simulator::Input().KeyTapped(SDLK_BACKQUOTE))
        this->m_view->ShowAll();
}

void RobotController::Poll()
{
    this->PollChangeIdx();

    if (this->m_robot->IsDKForwards())
        this->PollChangeQ();
    else
        this->PollChangeEndEffector();

    this->PollToggleShown();
    this->PollChangeCamera();
    this->PollChangeDKMode();

    this->m_robot->Recompute();
}

