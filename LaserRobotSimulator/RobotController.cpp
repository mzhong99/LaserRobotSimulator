#include "Robot.hpp"
#include <cmath> 
#include <SDL.h>

#include <iomanip>

#define INCREMENT_VELOCITY          (3.0)

#define LONGITUDE_SENSITIVITY       (0.0025)
#define LATITUDE_SENSITIVITY        (0.005)
#define EE_SENS    (0.005)

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

    if (Simulator::Input().KeyTapped(SDLK_7))
        this->m_robot->SelectIdx(6);

    if (Simulator::Input().KeyTapped(SDLK_8))
        this->m_robot->SelectIdx(7);

    if (Simulator::Input().KeyTapped(SDLK_9))
        this->m_robot->SelectIdx(8);
}

void RobotController::PollChangeFwdK()
{
    double oldQ = m_robot->GetFwdK().GetQ(m_robot->GetIdx());

    if (!Simulator::Input().KeyPressed(SDLK_LCTRL) && !Simulator::Input().KeyPressed(SDLK_LSHIFT))
    {
        if (Simulator::Input().MousePressed(SDL_BUTTON_LEFT))
        {
            Vector2D<int> motion = Simulator::Input().MouseMotion();
            m_robot->GetFwdK().SetQ(m_robot->GetIdx(), oldQ + (motion.y * -0.005));
        }

        if (Simulator::Input().KeyTapped(SDLK_0))
            this->m_robot->GetFwdK().SetQ(m_robot->GetIdx(), 0);
    }
}

void RobotController::PollSelectSimMode()
{
    if (Simulator::Input().KeyTapped(SDLK_ESCAPE))
        m_robot->SelectSimMode(SimulationMode::VIEW_ALL);

    if (Simulator::Input().KeyTapped(SDLK_t))
        m_robot->SelectSimMode(SimulationMode::STATICS);

    if (Simulator::Input().KeyTapped(SDLK_j))
        m_robot->SelectSimMode(SimulationMode::FWD_KINEMATICS);

    if (Simulator::Input().KeyTapped(SDLK_i))
        m_robot->SelectSimMode(SimulationMode::INV_KINEMATICS);

    if (Simulator::Input().KeyTapped(SDLK_v))
        m_robot->SelectSimMode(SimulationMode::NONE);

    if (Simulator::Input().KeyTapped(SDLK_g))
        m_robot->SelectSimMode(SimulationMode::PATH_PLAN);
}

void RobotController::PollChangeCamera()
{
    if (Simulator::Input().KeyPressed(SDLK_w))
        this->m_view->Camera().AccumulateForwards(1.0);

    if (Simulator::Input().KeyPressed(SDLK_s))
        this->m_view->Camera().AccumulateForwards(-1.0);

    if (Simulator::Input().KeyPressed(SDLK_a))
        this->m_view->Camera().AccumulateSideways(-1.0);

    if (Simulator::Input().KeyPressed(SDLK_d))
        this->m_view->Camera().AccumulateSideways(1.0);

    if (Simulator::Input().KeyPressed(SDLK_SPACE))
        this->m_view->Camera().AccumulateUpDown(1.0);

    if (Simulator::Input().KeyPressed(SDLK_c))
        this->m_view->Camera().AccumulateUpDown(-1.0);

    if (Simulator::Input().KeyTapped(SDLK_p))
        this->m_view->Camera().ToggleUsePerspective();

    if (Simulator::Input().KeyPressed(SDLK_EQUALS))
        this->m_view->Camera().ZoomIn();

    if (Simulator::Input().KeyPressed(SDLK_MINUS))
        this->m_view->Camera().ZoomOut();

    if (!Simulator::Input().KeyPressed(SDLK_LCTRL) && !Simulator::Input().KeyPressed(SDLK_LSHIFT))
    {
        if (Simulator::Input().MousePressed(SDL_BUTTON_MIDDLE))
        {
            Vector2D<int> motion = Simulator::Input().MouseMotion();
            this->m_view->Camera().AccumulateXRotation(-1.0 * motion.y * LONGITUDE_SENSITIVITY);
            this->m_view->Camera().AccumulateYRotation(motion.x * LATITUDE_SENSITIVITY);
        }
    }
}

void RobotController::PollToggleShown()
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

        if (Simulator::Input().KeyTapped(SDLK_7))
            this->m_view->ShowSolo(6);

        if (Simulator::Input().KeyTapped(SDLK_8))
            this->m_view->ShowSolo(7);

        if (Simulator::Input().KeyTapped(SDLK_9))
            this->m_view->ShowSolo(8);
    }
    else if (Simulator::Input().KeyPressed(SDLK_LCTRL))
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

        if (Simulator::Input().KeyTapped(SDLK_7))
            this->m_view->ToggleShown(6);

        if (Simulator::Input().KeyTapped(SDLK_8))
            this->m_view->ToggleShown(7);

        if (Simulator::Input().KeyTapped(SDLK_9))
            this->m_view->ToggleShown(8);
    }

    if (Simulator::Input().KeyTapped(SDLK_BACKQUOTE))
        this->m_view->ShowAll();
}

void RobotController::PollChangeStatics()
{
    if (Simulator::Input().KeyPressed(SDLK_LSHIFT))
    {
        if (Simulator::Input().MousePressed(SDL_BUTTON_RIGHT))
        {
            Vector2D<int> motion = Simulator::Input().MouseMotion();
            m_robot->GetStatics().AccEEForce(-1.0 * motion.y * LATITUDE_SENSITIVITY, 0, 0);
        }

        if (Simulator::Input().MousePressed(SDL_BUTTON_MIDDLE))
        {
            Vector2D<int> motion = Simulator::Input().MouseMotion();

            m_robot->GetStatics().AccEEForce(
                0, motion.y * -1.0 * LATITUDE_SENSITIVITY, motion.x * LONGITUDE_SENSITIVITY);
        }
    }

    if (Simulator::Input().KeyPressed(SDLK_LCTRL))
    {
        if (Simulator::Input().MousePressed(SDL_BUTTON_RIGHT))
        {
            Vector2D<int> motion = Simulator::Input().MouseMotion();
            m_robot->GetStatics().AccEEMoment(-1.0 * motion.y * LATITUDE_SENSITIVITY, 0, 0);
        }

        if (Simulator::Input().MousePressed(SDL_BUTTON_MIDDLE))
        {
            Vector2D<int> motion = Simulator::Input().MouseMotion();
            m_robot->GetStatics().AccEEMoment(
                0, motion.y * -1.0 * LATITUDE_SENSITIVITY, motion.x * LONGITUDE_SENSITIVITY);
        }
    }
}

void RobotController::PollChangePathPlanner()
{
    if (Simulator::Input().MousePressed(SDL_BUTTON_LEFT))
    {
        Vector2D<int> motion = Simulator::Input().MouseMotion();
        Vector3D<double> linDelta = 0;
        Vector3D<double> angDelta = 0;

        if (Simulator::Input().KeyPressed(SDLK_LALT))
        {
            if (Simulator::Input().KeyPressed(SDLK_LSHIFT))
                angDelta = Vector3D<double>(0, EE_SENS * -1.0 * motion.Y(), 0);
            else
            {
                Vector3D<double> forwardsUnit = m_view->Camera().ForwardsUnit();
                Vector3D<double> sidewaysUnit = m_view->Camera().SidewaysUnit();

                Vector3D<double> forwardsDelta = forwardsUnit * EE_SENS * -1.0 * (double)motion.Y();
                Vector3D<double> sidewaysDelta = sidewaysUnit * EE_SENS * (double)motion.X();

                angDelta = forwardsDelta + sidewaysDelta;
            }
        }
        else
        {
            if (Simulator::Input().KeyPressed(SDLK_LSHIFT))
                linDelta = Vector3D<double>(0, EE_SENS * -1.0 * motion.Y(), 0);
            else
            {
                Vector3D<double> forwardsUnit = m_view->Camera().ForwardsUnit();
                Vector3D<double> sidewaysUnit = m_view->Camera().SidewaysUnit();

                Vector3D<double> forwardsDelta = forwardsUnit * EE_SENS * -1.0 * (double)motion.Y();
                Vector3D<double> sidewaysDelta = sidewaysUnit * EE_SENS * (double)motion.X();

                linDelta = forwardsDelta + sidewaysDelta;
            }
        }

        m_robot->GetPlanner().AccLinPos(linDelta);
        m_robot->GetPlanner().AccAngPos(angDelta);
    }

    if (Simulator::Input().MousePressed(SDL_BUTTON_RIGHT))
    {
        Vector2D<int> motion = Simulator::Input().MouseMotion();
        m_robot->GetPlanner().AccTravelTime(motion.Y() * EE_SENS);
    }

    if (Simulator::Input().KeyTapped(SDLK_x))
        m_robot->GetPlanner().AdvanceState();
}

void RobotController::PollChangeInvK()
{
    if (Simulator::Input().MousePressed(SDL_BUTTON_LEFT))
    {
        Vector2D<int> motion = Simulator::Input().MouseMotion();
        if (Simulator::Input().KeyPressed(SDLK_LALT))
        {
            if (Simulator::Input().KeyPressed(SDLK_LSHIFT))
            {
                Vector3D<double> fullDelta = Vector3D<double>(0, 0, EE_SENS * -1.0 * motion.Y());
                m_robot->GetInvK().AccTargetEEPos(0, fullDelta);
            }
            else
            {
                double xDelta = EE_SENS * -1.0 * (double)motion.Y();
                double yDelta = EE_SENS * (double)motion.X();

                Vector3D<double> fullDelta = Vector3D<double>(xDelta, yDelta, 0);
                m_robot->GetInvK().AccTargetEEPos(0, fullDelta);
            }
        }
        else
        {
            if (Simulator::Input().KeyPressed(SDLK_LSHIFT))
            {
                Vector3D<double> fullDelta = Vector3D<double>(0, 0, EE_SENS * -1.0 * motion.Y());
                m_robot->GetInvK().AccTargetEEPos(fullDelta, 0);
            }
            else
            {
                Vector3D<double> forwardsUnit = m_view->Camera().ForwardsUnit();
                Vector3D<double> sidewaysUnit = m_view->Camera().SidewaysUnit();

                Vector3D<double> forwardsDelta = forwardsUnit * EE_SENS * -1.0 * (double)motion.Y();
                Vector3D<double> sidewaysDelta = sidewaysUnit * EE_SENS * (double)motion.X();

                Vector3D<double> fullDelta = forwardsDelta + sidewaysDelta;
                m_robot->GetInvK().AccTargetEEPos(fullDelta, 0);
            }
        }
    }
}

void RobotController::PollChangeInvDK()
{
    if (Simulator::Input().KeyPressed(SDLK_LSHIFT))
    {
        if (Simulator::Input().MousePressed(SDL_BUTTON_RIGHT))
        {
            Vector2D<int> motion = Simulator::Input().MouseMotion();
            m_robot->GetInvK().AccTargetEELinVel(-1.0 * motion.y * LATITUDE_SENSITIVITY, 0, 0);
        }

        if (Simulator::Input().MousePressed(SDL_BUTTON_MIDDLE))
        {
            Vector2D<int> motion = Simulator::Input().MouseMotion();
            m_robot->GetInvK().AccTargetEELinVel(
                0, motion.y * -1.0 * LATITUDE_SENSITIVITY, motion.x * LONGITUDE_SENSITIVITY);
        }
    }

    if (Simulator::Input().KeyPressed(SDLK_LCTRL))
    {
        if (Simulator::Input().MousePressed(SDL_BUTTON_RIGHT))
        {
            Vector2D<int> motion = Simulator::Input().MouseMotion();
            m_robot->GetInvK().AccTargetEEAngVel(-1.0 * motion.y * LATITUDE_SENSITIVITY, 0, 0);
        }

        if (Simulator::Input().MousePressed(SDL_BUTTON_MIDDLE))
        {
            Vector2D<int> motion = Simulator::Input().MouseMotion();
            m_robot->GetInvK().AccTargetEEAngVel(
                0, motion.y * -1.0 * LATITUDE_SENSITIVITY, motion.x * LONGITUDE_SENSITIVITY);
        }
    }
}

void RobotController::PollChangeFwdDK()
{
    if (!Simulator::Input().KeyPressed(SDLK_LCTRL) && !Simulator::Input().KeyPressed(SDLK_LSHIFT))
    {
        if (Simulator::Input().MousePressed(SDL_BUTTON_RIGHT))
        {
            Vector2D<int> motion = Simulator::Input().MouseMotion();
            m_robot->GetFwdK().AccQPrime(m_robot->GetIdx(), motion.y * -0.005);
        }
    }
}

void RobotController::Poll()
{
    this->PollChangeIdx();
    this->PollSelectSimMode();

    switch (this->m_robot->SimMode())
    {
        case SimulationMode::FWD_KINEMATICS:
            this->PollChangeFwdDK();
            this->PollChangeFwdK();
            break;

        case SimulationMode::INV_KINEMATICS:
            this->m_robot->SelectIdx(this->m_robot->NumJoints() - 1);
            this->PollChangeInvK();
            this->PollChangeInvDK();
            break;

        case SimulationMode::STATICS:
            this->PollChangeStatics();
            this->PollChangeFwdK();
            break;

        case SimulationMode::PATH_PLAN:
            this->PollChangePathPlanner();
            break;

        case SimulationMode::VIEW_ALL:
            this->PollChangeFwdK();
            break;

        default:
            break;
    }

    this->PollToggleShown();
    this->PollChangeCamera();

    this->m_robot->Recompute();
}
