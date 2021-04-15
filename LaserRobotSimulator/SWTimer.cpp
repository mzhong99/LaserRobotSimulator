#include "SWTimer.hpp"

SWTimer::SWTimer(uint32_t deltaTime_ms)
{
    this->startTicks = 0;
    this->waitTicks = deltaTime_ms;
    this->percentProgressOffset = 0;
}

void SWTimer::Start()
{
    this->startTicks = SDL_GetTicks();
}

bool SWTimer::Expired()
{
    uint32_t timeElapsed = this->ElapsedMS();
    return timeElapsed > (1.0 - this->percentProgressOffset) * this->waitTicks;
}

double SWTimer::PercentElapsed()
{
    double raw = (double)this->ElapsedMS() / (double)this->waitTicks 
        + this->percentProgressOffset;

    return fmin(1.0, raw);
}

uint32_t SWTimer::ElapsedMS()
{
    return SDL_GetTicks() - this->startTicks;
}

void SWTimer::SetProgress(double progress)
{
    this->percentProgressOffset = progress;
}
