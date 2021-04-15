#pragma once

#include <SDL.h>

class SWTimer
{
private:
    uint32_t startTicks;
    uint32_t waitTicks;
    double percentProgressOffset;

public:
    SWTimer(uint32_t deltaTime_ms = 1);
    void Start();
    bool Expired();
    double PercentElapsed();
    uint32_t ElapsedMS();
    void SetProgress(double progress);
};

