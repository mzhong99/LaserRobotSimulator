#pragma once

#include "Input.hpp"
#include "Graphics.hpp"
#include "Application.hpp"

class Simulator
{
private:
    static Input *input;
    static Graphics *graphics;
    static App *application;

public:
    static void Initialize();
    static void Teardown();

    static App &App();
    static Graphics &Graphics();
    static Input &Input();
};
