#include "Input.hpp"

void Input::KeyPushDown(SDL_Keycode key)
{
    this->keys[key].PushDown();
}

void Input::KeyPullUp(SDL_Keycode key)
{
    this->keys[key].PullUp();
}

bool Input::KeyTapped(SDL_Keycode key)
{
    return this->keys[key].Tapped();
}

bool Input::KeyPressed(SDL_Keycode key)
{
    return this->keys[key].Pressed();
}

void Input::MousePushDown(SDL_MouseButton mouse)
{
    this->mouseButtons[mouse].PushDown();
}

void Input::MousePullUp(SDL_MouseButton mouse)
{
    this->mouseButtons[mouse].PullUp();
}

void Input::MouseUpdatePosition(int x, int y)
{
    Vector2D<int> newPosition = Vector2D<int>(x, y);
    this->mouseMotion = newPosition - this->mousePosition;
    this->mousePosition = newPosition;

    this->mousePositionUpdated = true;
}

Vector2D<int> Input::MousePosition()
{
    return this->mousePosition;
}

Vector2D<int> Input::MouseMotion()
{
    return this->mouseMotion;
}

bool Input::MouseTapped(SDL_MouseButton mouse)
{
    return this->mouseButtons[mouse].Tapped();
}

bool Input::MousePressed(SDL_MouseButton mouse)
{
    return this->mouseButtons[mouse].Pressed();
}

void Input::PollCleanup()
{
    if (this->mousePositionUpdated)
        this->mouseMotion = Vector2D<int>(0, 0);

    this->mousePositionUpdated = false;

    for (auto it = this->keys.begin(); it != this->keys.end(); it++)
        it->second.Refresh();

    for (auto it = this->mouseButtons.begin(); it != this->mouseButtons.end(); it++)
        it->second.Refresh();
}
