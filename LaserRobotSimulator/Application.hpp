#pragma once

#include <SDL.h>
#include <cstddef>
#include "SWTimer.hpp"
#include "DebugOverlay.hpp"
#include "RobotFwd.hpp"

#define FRAMERATE_CAP   60

class Application
{
private:
    DebugOverlay debugOverlay;

    SWTimer deltaTimer;
    uint32_t deltaTime_ms;

    Robot *m_robot;
    RobotController *m_controller;
    RobotView *m_view;

    bool exited;

    void ReadSystemEvents();

public:
    Application();
    ~Application();

    Application(const Application &) = delete;

    void Poll();

    bool Exited() { return this->exited; }
    uint32_t DeltaTimeMS() { return this->deltaTime_ms; }
};

