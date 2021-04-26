#pragma once

#include <SDL.h>
#include <mutex>

class SWTimer
{
private:
    uint64_t m_startTicks;
    double m_waitTime;

    static std::mutex s_hwCounterLock;

public:
    SWTimer(double waitTimeSeconds = 0);
    void Start();
    bool Expired();
    double ElapsedSeconds();
};

