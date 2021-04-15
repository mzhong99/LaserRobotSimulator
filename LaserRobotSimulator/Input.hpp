#pragma once

#include <unordered_map>
#include <SDL.h>

#include "Vector2D.hpp"

typedef uint8_t SDL_MouseButton;

class InputState
{
private:
    bool tapped = false;
    bool pressed = false;

    bool tapAccepted = false;

public:
    void PushDown()
    {
        this->tapped = true;
        this->pressed = true;
    }

    bool Tapped()
    {
        if (this->tapped)
            this->tapAccepted = true;

        return this->tapAccepted;
    }

    bool Pressed()
    {
        return this->pressed;
    }

    void PullUp()
    {
        this->tapped = false;
        this->pressed = false;
        this->tapAccepted = false;
    }

    void Refresh()
    {
        if (this->tapAccepted)
        {
            this->tapAccepted = false;
            this->tapped = false;
        }
    }
};

class Input
{
private:
    std::unordered_map<SDL_Keycode, InputState> keys;
    std::unordered_map<SDL_MouseButton, InputState> mouseButtons;
    Vector2D<int> mousePosition;
    Vector2D<int> mouseMotion;
    bool mousePositionUpdated = false;

public:
    Input() {}

    void KeyPushDown(SDL_Keycode key);
    void KeyPullUp(SDL_Keycode key);

    bool KeyTapped(SDL_Keycode key);
    bool KeyPressed(SDL_Keycode key);
    
    void MousePushDown(SDL_MouseButton mouse);
    void MousePullUp(SDL_MouseButton mouse);
    void MouseUpdatePosition(int x, int y);
    Vector2D<int> MousePosition();
    Vector2D<int> MouseMotion();

    bool MouseTapped(SDL_MouseButton mouse);
    bool MousePressed(SDL_MouseButton mouse);

    void PollCleanup();
};

