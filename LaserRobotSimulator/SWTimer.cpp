#include "SWTimer.hpp"

std::mutex SWTimer::s_hwCounterLock = std::mutex();

SWTimer::SWTimer(double waitTimeSeconds)
{
    this->m_startTicks = 0;
    this->m_waitTime = waitTimeSeconds;
}

void SWTimer::Start()
{
    std::lock_guard<std::mutex> hwLock(SWTimer::s_hwCounterLock);
    this->m_startTicks = SDL_GetPerformanceCounter();
}

bool SWTimer::Expired()
{
    return this->ElapsedSeconds() > m_waitTime;
}

double SWTimer::ElapsedSeconds()
{
    std::lock_guard<std::mutex> hwLock(SWTimer::s_hwCounterLock);
    return (SDL_GetPerformanceCounter() - m_startTicks) / (double)SDL_GetPerformanceFrequency();
}

