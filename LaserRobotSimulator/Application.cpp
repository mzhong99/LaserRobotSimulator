#include "Application.hpp"
#include "Simulator.hpp"
#include "Robot.hpp"

App::App()
{
    this->deltaTimer = SWTimer(0);
    this->debugOverlay = DebugOverlay();

    this->exited = false;

    this->deltaTime_ms = 0;
    this->deltaTimer.Start();

    this->m_robot = new Robot();
    this->m_view = new RobotView(this->m_robot);
    this->m_controller = new RobotController(this->m_robot, this->m_view);
}

App::~App()
{
    delete this->m_robot;
    delete this->m_view;
    delete this->m_controller;
}

void App::ReadSystemEvents()
{
    SDL_Event event;
    while (SDL_PollEvent(&event) != 0)
    {
        switch (event.type)
        {
            case SDL_QUIT:
                this->exited = true;
                break;

            case SDL_KEYDOWN:
                Simulator::Input().KeyPushDown(event.key.keysym.sym);
                break;

            case SDL_KEYUP:
                Simulator::Input().KeyPullUp(event.key.keysym.sym);
                break;

            case SDL_MOUSEBUTTONDOWN:
                Simulator::Input().MousePushDown(event.button.button);
                break;

            case SDL_MOUSEBUTTONUP:
                Simulator::Input().MousePullUp(event.button.button);
                break;

            case SDL_MOUSEMOTION:
                Simulator::Input().MouseUpdatePosition(event.motion.x, event.motion.y);
                break;

            default:
                break;
        }
    }
}

void App::Poll()
{
    this->ReadSystemEvents();
    this->debugOverlay.Poll();

    this->m_controller->Poll();
    this->m_view->Poll();
    
    Simulator::Graphics().SwapBuffer();
    Simulator::Input().PollCleanup();

    while (this->deltaTimer.ElapsedMS() < 1000 / FRAMERATE_CAP);
    this->deltaTime_ms = this->deltaTimer.ElapsedMS();
    this->deltaTimer.Start();
}

