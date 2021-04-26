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

void RobotView::ShowJointTable(std::vector<DHParam> dhparams)
{
    Vector2D<int> offset = DK_SCREEN_OFFSET;
    Simulator::Graphics().PrintString(offset.x, offset.y, "Joint (DH) Table [%s]",
        m_robot->SimMode() == SimulationMode::NONE ? "Kinematics" :
        m_robot->SimMode() == SimulationMode::STATICS ? "Statics" :
        m_robot->SimMode() == SimulationMode::FWD_KINEMATICS ? "Differential Kinematics" :
        m_robot->SimMode() == SimulationMode::VIEW_ALL ? "View All" : "");

    for (size_t i = 0; i < this->m_robot->NumJoints(); i++)
    {
        if (i == this->m_robot->GetIdx())
            Simulator::Graphics().SetFGColor(Graphics::COLOR_BLUE);

        Simulator::Graphics().PrintString(offset.x, offset.y + (12 * (i + 1)),
            "    Q%d = %+3.3f    Th: %+.3f  Alp: %+.3f  A: %+.3f  D:%+.3f",
            (int)i + 1, dhparams[i].GetQ(),
            dhparams[i].Theta, dhparams[i].Alpha, dhparams[i].A, dhparams[i].D);

        if (i == this->m_robot->GetIdx())
        {
            Simulator::Graphics().PrintString(offset.x, offset.y + (12 * ((int)i + 1)), " -> ");
            Simulator::Graphics().SetFGColor(Graphics::COLOR_WHITE);
        }
    }
}

void RobotView::ShowEEPosition(Vector3D<double> eeLinPos, Vector3D<double> eeAngPos)
{
    Vector2D<int> offset = DK_SCREEN_OFFSET;

    Simulator::Graphics().PrintString(offset.x, offset.y + 600, "End Effector Position");

    Simulator::Graphics().PrintString(offset.x, offset.y + 612,
        "     Linear: <lx=%+3.3f, ly=%+3.3f, lz=%+3.3f> m", eeLinPos.x, eeLinPos.y, eeLinPos.z);

    Simulator::Graphics().PrintString(offset.x, offset.y + 624,
        "    Angular: <ax=%+3.3f, ay=%+3.3f, az=%+3.3f> rad", eeAngPos.x, eeAngPos.y, eeAngPos.z);
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

void RobotView::ShowInvK()
{
    CoordinateStub baseFrame;

    this->ShowInvJointTable();
    this->ShowInvEEPosition();
    
    this->RenderStub(baseFrame, false, 0);
    std::vector<CoordinateStub> stubs = this->m_robot->GetInvK().GetStubs();

    for (size_t i = 0; i < stubs.size(); i++)
        if (this->m_shown[i])
            this->RenderStub(stubs[i], (int)i == this->m_robot->GetIdx(), i + 1);
}

void RobotView::ShowInvDK()
{
    std::vector<CoordinateStub> stubs = m_robot->GetInvK().GetStubs();
    std::vector<DHParam> dhparams = m_robot->GetAllDHParams();
    std::vector<double> velocities = m_robot->GetInvK().GetQPrime();

    CoordinateStub prevStub = CoordinateStub();

    for (size_t i = 0; i < stubs.size(); i++)
    {
        if (this->m_shown[i])
            this->RenderJointVelocity(prevStub, velocities[i], dhparams[i].Type, i + 1);

        prevStub = stubs[i];
    }

    /* Show the velocity vector on the end effector */
    Transform stubToBase = m_robot->GetInvK().TransformEEToBase();
    Vector3D<double> ori3D = m_robot->GetInvK().EELinPos();

    Vector3D<double> linV = m_robot->GetInvK().EELinVel();
    Vector3D<double> angV = m_robot->GetInvK().EEAngVel();

    Vector3D<double> linPos = ori3D + linV;
    Vector3D<double> angPos = ori3D + angV;

    Simulator::Graphics().SetFGColor(Graphics::COLOR_LIGHTGRAY);
    this->m_camera.DrawArrow(ori3D, linPos, "V_e=<%.3f, %.3f, %.3f> m/s", linV.x, linV.y, linV.z);
    this->m_camera.DrawArrow(ori3D, angPos, "W_e=<%.3f, %.3f, %.3f> rad/s", angV.x, angV.y, angV.z);
    Simulator::Graphics().SetFGColor(Graphics::COLOR_WHITE);
}

void RobotView::ShowFwdK()
{
    CoordinateStub baseFrame;

    this->ShowFwdJointTable();
    this->ShowFwdEEPosition();
    
    this->RenderStub(baseFrame, false, 0);
    std::vector<CoordinateStub> &stubs = this->m_robot->GetFwdK().Stubs();

    for (size_t i = 0; i < stubs.size(); i++)
        if (this->m_shown[i])
            this->RenderStub(stubs[i], (int)i == this->m_robot->GetIdx(), i + 1);
}

void RobotView::ShowFwdDK()
{
    std::vector<CoordinateStub> &stubs = m_robot->GetFwdK().Stubs();
    std::vector<DHParam> dhparams = m_robot->GetAllDHParams();
    std::vector<double> &velocities = m_robot->GetFwdK().QPrime();

    CoordinateStub prevStub = CoordinateStub();

    for (size_t i = 0; i < stubs.size(); i++)
    {
        if (this->m_shown[i])
            this->RenderJointVelocity(prevStub, velocities[i], dhparams[i].Type, i + 1);

        prevStub = stubs[i];
    }

    /* Show the velocity vector on the end effector */
    Transform stubToBase = m_robot->GetFwdK().TransformEEToBase();
    Vector3D<double> ori3D = m_robot->GetFwdK().EELinPos();

    Vector3D<double> linV = m_robot->GetFwdK().EELinVel();
    Vector3D<double> angV = m_robot->GetFwdK().EEAngVel();

    Vector3D<double> linPos = ori3D + linV;
    Vector3D<double> angPos = ori3D + angV;

    Simulator::Graphics().SetFGColor(Graphics::COLOR_LIGHTGRAY);
    this->m_camera.DrawArrow(ori3D, linPos, "V_e=<%.3f, %.3f, %.3f> m/s", linV.x, linV.y, linV.z);
    this->m_camera.DrawArrow(ori3D, angPos, "W_e=<%.3f, %.3f, %.3f> rad/s", angV.x, angV.y, angV.z);
    Simulator::Graphics().SetFGColor(Graphics::COLOR_WHITE);
}

void RobotView::ShowEEStatics()
{
    Transform stubToBase = m_robot->GetFwdK().TransformEEToBase();
    Vector3D<double> origin3D = m_robot->GetFwdK().EELinPos();

    Vector3D<double> force = m_robot->GetStatics().EEFMCouple().Force();
    Vector3D<double> moment = m_robot->GetStatics().EEFMCouple().Moment();

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
    std::vector<DHParam> dhparams = m_robot->GetAllDHParams();
    std::vector<double> &reactions = m_robot->GetStatics().QReactions();

    CoordinateStub prevStub = CoordinateStub();

    for (size_t i = 0; i < m_robot->GetFwdK().Stubs().size(); i++)
    {
        if (this->m_shown[i])
            this->RenderJointStaticReaction(prevStub, reactions[i], dhparams[i].Type, i + 1);

        prevStub = m_robot->GetFwdK().Stubs()[i];
    }
}

void RobotView::ShowBaseTransformStatics()
{
    Vector3D<double> bEqForce = m_robot->GetStatics().BEqForce();
    Vector3D<double> bEqMoment = m_robot->GetStatics().BEqMoment();

    Vector3D<double> origin3D = Vector3D<double>(0, 0, 0); /* because it's literally the origin */

    Simulator::Graphics().SetFGColor(Graphics::COLOR_CYAN);

    this->m_camera.DrawArrow(origin3D, bEqForce,
        "F_eq=<%.3lf, %.3lf, %.3lf> N", bEqForce.x, bEqForce.y, bEqForce.z);
    this->m_camera.DrawArrow(origin3D, bEqMoment,
        "M_eq=<%.3lf, %.3lf, %.3lf> Nm", bEqMoment.x, bEqMoment.y, bEqMoment.z);
    Simulator::Graphics().SetFGColor(Graphics::COLOR_WHITE);
}

void RobotView::ShowStatics()
{
    this->ShowEEStatics();
    this->ShowJointStatics();
    this->ShowBaseTransformStatics();
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
    m_camera.RefreshView();

    switch (this->m_robot->SimMode())
    {
        case SimulationMode::NONE:
            this->ShowFwdK();
            break;

        case SimulationMode::FWD_KINEMATICS:
            this->ShowFwdK();
            this->ShowFwdDK();
            break;

        case SimulationMode::INV_KINEMATICS:
            this->ShowInvK();
            this->ShowInvDK();
            break;

        case SimulationMode::STATICS:
            this->ShowFwdK();
            this->ShowStatics();
            break;

        case SimulationMode::VIEW_ALL:
            this->ShowFwdK();
            this->ShowFwdDK();
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
