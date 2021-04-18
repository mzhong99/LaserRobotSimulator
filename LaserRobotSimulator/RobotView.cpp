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
}

void RobotView::ShowJointTable()
{
    Vector2D<int> offset = DK_SCREEN_OFFSET;
    Simulator::Graphics().PrintString(offset.x, offset.y, "Joint (DH) Table [%s] [%s]",
        m_robot->SimMode() == SimulationMode::NONE                    ? "Kinematics" :
        m_robot->SimMode() == SimulationMode::STATICS                 ? "Statics" :
        m_robot->SimMode() == SimulationMode::DIFFERENTIAL_KINEMATICS ? "Differential Kinematics" :
        m_robot->SimMode() == SimulationMode::VIEW_ALL                ? "View All" : "",
        m_robot->IsSimForwards() ? "Forwards" : "Inverse");

    std::vector<DHParam> dhparams = this->m_robot->GetAllDHParams();

    for (size_t i = 0; i < this->m_robot->NumJoints(); i++)
    {
        if (i == this->m_robot->GetIdx())
            Simulator::Graphics().SetFGColor(Graphics::COLOR_BLUE);

        Simulator::Graphics().PrintString(offset.x, offset.y + (12 * (i + 1)),
            "    Q%d = %+3.3f  %s  Th: %+.3f  Alp: %+.3f  A: %+.3f  D:%+.3f",
            (int)i + 1, dhparams[i].GetQ(),
            (!m_robot->IsSimForwards() && m_robot->IsFreeJoint(i)) ? "[Free]" : "      ",
            dhparams[i].Theta, dhparams[i].Alpha, dhparams[i].A, dhparams[i].D);

        if (i == this->m_robot->GetIdx())
        {
            Simulator::Graphics().PrintString(offset.x, offset.y + (12 * ((int)i + 1)), " -> ");
            Simulator::Graphics().SetFGColor(Graphics::COLOR_WHITE);
        }
    }
}

void RobotView::ShowEEPosition()
{
    Vector2D<int> offset = DK_SCREEN_OFFSET;

    Simulator::Graphics().PrintString(offset.x, offset.y + 600, "End Effector Position");

    Vector3D<double> eeLinearPosition = this->m_robot->GetEndEffectorPosition();
    Vector3D<double> eeAngularPosition = this->m_robot->GetEndEffectorAngularPosition();

    Simulator::Graphics().PrintString(offset.x, offset.y + 612,
        "     Linear: <lx=%+3.3f, ly=%+3.3f, lz=%+3.3f> m",
        eeLinearPosition.x, eeLinearPosition.y, eeLinearPosition.z);

    Simulator::Graphics().PrintString(offset.x, offset.y + 624,
        "    Angular: <ax=%+3.3f, ay=%+3.3f, az=%+3.3f> rad",
        eeAngularPosition.x, eeAngularPosition.y, eeAngularPosition.z);
}

void RobotView::RenderJointVelocity(
    CoordinateStub &stub, double velocity, JointType jointType, int frameNumber)
{
    Transform stubToBase = stub.GetTransform();

    Vector3D<double> origin3D = stubToBase.GetColumn(3);
    Vector3D<double> offset3D = stubToBase.GetColumn(2) * velocity;

    Vector3D<double> rxnPoint3D = origin3D + offset3D;

    Simulator::Graphics().SetFGColor(Graphics::COLOR_GRAY);
    this->m_camera.DrawArrow(origin3D, rxnPoint3D,
        "%s_Q%d'=%.3lf%s",
        jointType == JointType::PRISMATIC ? "V" : "W",
        frameNumber, velocity,
        jointType == JointType::PRISMATIC ? "m/s" : "rad/s");
    Simulator::Graphics().SetFGColor(Graphics::COLOR_WHITE);
}

void RobotView::ShowForwardsDK()
{
    std::vector<CoordinateStub> stubs = this->m_robot->GetAllStubs();
    std::vector<DHParam> dhparams = this->m_robot->GetAllDHParams();
    std::vector<double> velocities = this->m_robot->GetManualJointSpeeds();

    CoordinateStub prevStub = CoordinateStub();

    for (size_t i = 0; i < stubs.size(); i++)
    {
        if (this->m_shown[i])
            this->RenderJointVelocity(prevStub, velocities[i], dhparams[i].Type, i + 1);

        prevStub = stubs[i];
    }

    /* Show the velocity vector on the end effector */
    std::vector<double> eeVelocity = this->m_robot->GetForwardDKResults();

    Transform stubToBase = this->m_robot->EndEffectorTransform();
    Vector3D<double> origin3D = this->m_robot->GetEndEffectorPosition();

    Vector3D<double> linear = Vector3D<double>(eeVelocity[0], eeVelocity[1], eeVelocity[2]);
    Vector3D<double> angular = Vector3D<double>(eeVelocity[3], eeVelocity[4], eeVelocity[5]);

    Vector3D<double> linearPos = linear + origin3D;
    Vector3D<double> angularPos = angular + origin3D;

    Simulator::Graphics().SetFGColor(Graphics::COLOR_LIGHTGRAY);
    this->m_camera.DrawArrow(origin3D, linearPos,
        "V_e=<%.3f, %.3f, %.3f> m/s", linear.x, linear.y, linear.z);

    this->m_camera.DrawArrow(origin3D, angularPos,
        "W_e=<%.3f, %.3f, %.3f> rad/s", angular.x, angular.y, angular.z);
    Simulator::Graphics().SetFGColor(Graphics::COLOR_WHITE);
}

void RobotView::ShowInverseDK()
{
    std::vector<CoordinateStub> stubs = this->m_robot->GetAllStubs();
    std::vector<DHParam> dhparams = this->m_robot->GetAllDHParams();
    std::vector<double> jointVelocities = this->m_robot->GetInverseDKResults();

    if (jointVelocities.empty())
    {
        Vector2D<int> offset = DK_SCREEN_OFFSET;
        Simulator::Graphics().PrintString(offset.x, offset.y + (12 * (m_robot->NumJoints() + 1)),
            "    -> Jacobian is singular.");
        return;
    }

    CoordinateStub prevStub = CoordinateStub();

    for (size_t i = 0; i < stubs.size(); i++)
    {
        if (this->m_shown[i])
            this->RenderJointVelocity(prevStub, jointVelocities[i], dhparams[i].Type, i + 1);

        prevStub = stubs[i];
    }

    /* Drawing end effector velocity and acceleration */
    Vector3D<double> origin3D = this->m_robot->GetEndEffectorPosition();

    VelocityCouple eeVelocity = this->m_robot->GetEndEffectorVelocity();
    Vector3D<double> eeLinearVelocity = eeVelocity.Linear().Cartesian();
    Vector3D<double> eeAngularVelocity = eeVelocity.Angular().Cartesian();

    Vector3D<double> eeLinearVelocityPos = eeLinearVelocity + origin3D;
    Vector3D<double> eeAngularVelocityPos = eeAngularVelocity + origin3D;

    Simulator::Graphics().SetFGColor(Graphics::COLOR_LIGHTGRAY);
    this->m_camera.DrawArrow(origin3D, eeLinearVelocityPos,
        "V_e=<%.3f, %.3f, %.3f> m/s",
        eeLinearVelocity.x, eeLinearVelocity.y, eeLinearVelocity.z);

    this->m_camera.DrawArrow(origin3D, eeAngularVelocityPos,
        "W_e=<%.3f, %.3f, %.3f> rad/s",
        eeAngularVelocity.x, eeAngularVelocity.y, eeAngularVelocity.z);

    Simulator::Graphics().SetFGColor(Graphics::COLOR_WHITE);
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
    this->ShowEEStatics();
    this->ShowJointStatics();
    this->ShowBaseTransformStatics();
}

void RobotView::ShowDK()
{
    if (this->m_robot->IsSimForwards())
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
    this->m_camera.DrawArrow(origin3D, xHat3D, "x%u", (int)frameNumber);
    this->m_camera.DrawArrow(origin3D, yHat3D, "y%u", (int)frameNumber);

    Simulator::Graphics().SetFGColor(Graphics::COLOR_GREEN);
    this->m_camera.DrawArrow(origin3D, zHat3D, "z%u", (int)frameNumber);
    Simulator::Graphics().SetFGColor(Graphics::COLOR_WHITE);
}

void RobotView::ToggleShown(size_t idx)
{
    if (idx < this->m_shown.size())
        this->m_shown[idx] = !this->m_shown[idx];
}

void RobotView::Poll()
{
    CoordinateStub baseFrame;
    this->m_camera.RefreshView();

    this->ShowJointTable();
    this->ShowEEPosition();
    
    this->RenderStub(baseFrame, false, 0);
    std::vector<CoordinateStub> stubs = this->m_robot->GetAllStubs();

    for (size_t i = 0; i < stubs.size(); i++)
        if (this->m_shown[i])
            this->RenderStub(stubs[i], (int)i == this->m_robot->GetIdx(), i + 1);


    switch (this->m_robot->SimMode())
    {
        case SimulationMode::DIFFERENTIAL_KINEMATICS:
            this->ShowDK();
            break;

        case SimulationMode::STATICS:
            this->ShowStatics();
            break;

        case SimulationMode::VIEW_ALL:
            this->ShowDK();
            this->ShowStatics();
            break;

        default:
            break;
    }
}

void RobotView::ShowSolo(size_t idx)
{
    std::fill(this->m_shown.begin(), this->m_shown.end(), false);

    if (idx < this->m_robot->NumJoints())
    {
        this->m_shown[idx] = true;
        if (idx > 0)
            this->m_shown[idx - 1] = true;
    }
}
