#pragma once

#include "SWTimer.hpp"

class DebugOverlay
{
private:
    bool m_enabled;

public:
    DebugOverlay();
    void Poll();
};

