#include "Robot.hpp"
#include <cmath> 
#include <SDL.h>

#include <iomanip>

#define INCREMENT_VELOCITY      (3.0)

#define LONGITUDE_SENSITIVITY   (0.0025)
#define LATITUDE_SENSITIVITY    (0.005)

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

void RobotController::PollChangeQ()
{
    double scaler = 0.0;
    if (Simulator::Input().KeyPressed(SDLK_UP))
        scaler += 1.0;

    if (Simulator::Input().KeyPressed(SDLK_DOWN))
        scaler -= 1.0;

    double deltaTimeSeconds = Simulator::App().DeltaTimeMS() / 1000.0;
    double increment = scaler * INCREMENT_VELOCITY * deltaTimeSeconds;
    double oldQ = this->m_robot->GetJoint().GetQ();

    this->m_robot->GetJoint().SetQ(oldQ + increment);

    if (!Simulator::Input().KeyPressed(SDLK_LCTRL))
    {
        if (Simulator::Input().MousePressed(SDL_BUTTON_RIGHT))
        {
            Vector2D<int> motion = Simulator::Input().MouseMotion();
            this->m_robot->AccumulateJointSpeed(motion.y * -0.005);
        }

        if (Simulator::Input().MousePressed(SDL_BUTTON_LEFT))
        {
            Vector2D<int> motion = Simulator::Input().MouseMotion();
            this->m_robot->GetJoint().SetQ(oldQ + (motion.y * -0.005));
        }
    }
}

void RobotController::PollChangeEndEffector()
{
    if (Simulator::Input().MousePressed(SDL_BUTTON_LEFT) 
        && Simulator::Input().KeyPressed(SDLK_LCTRL))
    {
        Vector2D<int> motion = Simulator::Input().MouseMotion();
        this->m_robot->AccumulateEndEffectorVelocity(motion.y * -0.005);
    }
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

void RobotController::PollChangeDKMode()
{
    if (Simulator::Input().KeyTapped(SDLK_q))
        this->m_robot->ToggleDKForwards();
}

void RobotController::PollToggleShown()
{
    if (Simulator::Input().KeyTapped(SDLK_t))
        this->m_view->ToggleShowStatics();

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
        }
    }

    if (Simulator::Input().KeyTapped(SDLK_BACKQUOTE))
        this->m_view->ShowAll();
}

void RobotController::PollControlEEStatics()
{
    if (Simulator::Input().KeyPressed(SDLK_LSHIFT))
    {
        if (Simulator::Input().MousePressed(SDL_BUTTON_RIGHT))
        {
            Vector2D<int> motion = Simulator::Input().MouseMotion();
            this->m_robot->AccumulateEEForceMagnitude(-1.0 * motion.y * LATITUDE_SENSITIVITY);
        }

        if (Simulator::Input().MousePressed(SDL_BUTTON_MIDDLE))
        {
            Vector2D<int> motion = Simulator::Input().MouseMotion();

            this->m_robot->AccumulateEEForceOrientation(
                motion.y * -1.0 * LATITUDE_SENSITIVITY,
                motion.x * LONGITUDE_SENSITIVITY);
        }
    }

    if (Simulator::Input().KeyPressed(SDLK_LCTRL))
    {
        if (Simulator::Input().MousePressed(SDL_BUTTON_RIGHT))
        {
            Vector2D<int> motion = Simulator::Input().MouseMotion();
            this->m_robot->AccumulateEEMomentMagnitude(-1.0 * motion.y * LATITUDE_SENSITIVITY);
        }

        if (Simulator::Input().MousePressed(SDL_BUTTON_MIDDLE))
        {
            Vector2D<int> motion = Simulator::Input().MouseMotion();

            this->m_robot->AccumulateEEMomentOrientation(
                motion.y * -1.0 * LATITUDE_SENSITIVITY,
                motion.x * LONGITUDE_SENSITIVITY);
        }
    }
}

void RobotController::Poll()
{
    this->PollChangeIdx();

    this->PollChangeQ();
    if (!this->m_robot->IsDKForwards())
        this->PollChangeEndEffector();

    this->PollToggleShown();
    this->PollChangeCamera();
    this->PollChangeDKMode();

    this->PollControlEEStatics();

    this->m_robot->Recompute();
}
