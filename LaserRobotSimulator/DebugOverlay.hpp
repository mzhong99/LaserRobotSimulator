#pragma once

#include "SWTimer.hpp"

class DebugOverlay
{
private:
    bool enabled;
    SWTimer timer;

public:
    DebugOverlay();
    void Poll();
};

