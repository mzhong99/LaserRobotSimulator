#pragma once

#include <SDL.h>
#include <cstddef>
#include "SWTimer.hpp"
#include "DebugOverlay.hpp"
#include "RobotFwd.hpp"

#define FRAMERATE_CAP   60

class App
{
private:
    DebugOverlay debugOverlay;

    SWTimer m_deltaTimer;
    double m_deltaTime;

    Robot *m_robot;
    RobotController *m_controller;
    RobotView *m_view;

    bool exited;

    void ReadSystemEvents();

public:
    App();
    ~App();

    App(const App &) = delete;

    void Poll();

    bool Exited() { return this->exited; }
    double DeltaTimeSeconds() { return m_deltaTime; }
};

