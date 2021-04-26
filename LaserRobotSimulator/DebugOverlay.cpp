#include "DebugOverlay.hpp"
#include "Simulator.hpp"
#include "Utility.hpp"

#include <iostream>
#include <cmath>

DebugOverlay::DebugOverlay()
{
    this->m_enabled = true;
}

void DebugOverlay::Poll()
{
    Vector2D<int> mousePos;

    if (Simulator::Input().KeyTapped(SDLK_F3))
        this->m_enabled = !this->m_enabled;

    if (this->m_enabled)
        Simulator::Graphics().PrintString(
            0, 0, " FPS: %.2f", round(1.0 / Simulator::App().DeltaTimeSeconds()));

    if (Simulator::Input().KeyTapped(SDLK_F11))
        Simulator::Graphics().ToggleFullScreeen();
}
