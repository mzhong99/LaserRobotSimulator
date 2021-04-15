#include <SDL.h>
#include <SDL_image.h>
#include <SDL_ttf.h>

#include <filesystem>
#include <iostream>
#include <fstream>

#include "JSON.hpp"

#include <time.h>
#include <stdlib.h>

#include "SWTimer.hpp"
#include "Simulator.hpp"

#include "Application.hpp"
#include "Graphics.hpp"
#include "Input.hpp"

#include "Transform.hpp"

/**************************************************************************************************/
/** Gauntlet Overall Class Implementation ------------------------------------------------------- */
/**************************************************************************************************/
Input *Simulator::input;
Graphics *Simulator::graphics;
Application *Simulator::application;

void Simulator::Initialize()
{
    SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER);
    SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "0");
    SDL_Log("SDL Core Initialized.");

    TTF_Init();
    SDL_Log("TTF Extension Initialized.");

    IMG_Init(IMG_INIT_PNG);
    SDL_Log("IMG Extension Initialized.");

    Simulator::application = new ::Application();
    Simulator::graphics = new ::Graphics();
    Simulator::input = new ::Input();
    SDL_Log("System Initialized.");
}

void Simulator::Teardown()
{
    SDL_Log("Exiting System.");
    delete Simulator::graphics;
    delete Simulator::application;
    delete Simulator::input;

    SDL_Log("Exiting TTF.");
    TTF_Quit();

    SDL_Log("Exiting IMG.");
    IMG_Quit();

    SDL_Log("Exiting SDL Core.");
    SDL_Quit();
}

Application &Simulator::Application()
{
    return *Simulator::application;
}

Graphics &Simulator::Graphics()
{
    return *Simulator::graphics;
}

Input &Simulator::Input()
{
    return *Simulator::input;
}

/**************************************************************************************************/
/** Main Boilerplate Functions ------------------------------------------------------------------ */
/**************************************************************************************************/

int main(int argc, char *argv[])
{
    Simulator::Initialize();

    while (!Simulator::Application().Exited())
        Simulator::Application().Poll();

    Simulator::Teardown();

    return EXIT_SUCCESS;
}
