#include "DebugOverlay.hpp"
#include "Simulator.hpp"
#include "Utility.hpp"

#include <iostream>

DebugOverlay::DebugOverlay()
{
    this->enabled = false;

    this->timer = SWTimer(3000);
    this->timer.Start();
}

void DebugOverlay::Poll()
{
    Vector2D<int> mousePos;

    if (Simulator::Input().KeyTapped(SDLK_F3))
        this->enabled = !this->enabled;

    if (this->enabled)
    {
        mousePos = Simulator::Input().MouseMotion();

        Simulator::Graphics().PrintString(0, 0, " DT: %i", Simulator::App().DeltaTimeMS());
        Simulator::Graphics().PrintString(0, 12, " Cursor: %d %d", mousePos.X(), mousePos.Y());

        Simulator::Graphics().PrintString(0, 24, " M123: %d %d %d",
            Simulator::Input().MousePressed(SDL_BUTTON_LEFT),
            Simulator::Input().MousePressed(SDL_BUTTON_RIGHT),
            Simulator::Input().MousePressed(SDL_BUTTON_MIDDLE));

        Simulator::Graphics().PrintString(0, 36, " WASD: %d %d %d %d",
            Simulator::Input().KeyPressed(SDLK_w),
            Simulator::Input().KeyPressed(SDLK_a),
            Simulator::Input().KeyPressed(SDLK_s),
            Simulator::Input().KeyPressed(SDLK_d));

        if (this->timer.Expired())
            this->timer.Start();
    }

    if (Simulator::Input().KeyTapped(SDLK_F11))
        Simulator::Graphics().ToggleFullScreeen();
}
