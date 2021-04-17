#include "Graphics.hpp"
#include "Utility.hpp"
#include "Vector2D.hpp"

#include <cstdio>
#include <cstdarg>

#include <filesystem>
#include <iostream>
#include <string>

const SDL_Color Graphics::COLOR_BLACK = { 0x00, 0x00, 0x00, 0xFF };
const SDL_Color Graphics::COLOR_WHITE = { 0xFF, 0xFF, 0xFF, 0xFF };
const SDL_Color Graphics::COLOR_RED = { 0xFF, 0x00, 0x00, 0xFF };
const SDL_Color Graphics::COLOR_GREEN = { 0x00, 0xA0, 0x00, 0xFF };
const SDL_Color Graphics::COLOR_YELLOW = { 0xF0, 0xF0, 0x00, 0xFF };
const SDL_Color Graphics::COLOR_BLUE = { 0xA0, 0xA0, 0xFF, 0xFF };
const SDL_Color Graphics::COLOR_CYAN = { 0x00, 0xFF, 0xFF, 0xFF };
const SDL_Color Graphics::COLOR_GRAY = { 0x80, 0x80, 0x80, 0xFF };

#define TIP_LENGTH  (0.08)
#define TIP_LABEL_LENGTH  (0.05)

Graphics::Graphics()
{
    this->window = SDL_CreateWindow(
        "Robot Simulator",
        SDL_WINDOWPOS_CENTERED,
        SDL_WINDOWPOS_CENTERED,
        Graphics::DEFAULT_WIDTH,
        Graphics::DEFAULT_HEIGHT,
        SDL_WINDOW_RESIZABLE);

    this->renderer = SDL_CreateRenderer(this->window, -1, SDL_RENDERER_ACCELERATED);
    this->fgcolor = Graphics::COLOR_WHITE;
    this->bgcolor = Graphics::COLOR_BLACK;

    SDL_RenderSetLogicalSize(this->renderer, Graphics::DEFAULT_WIDTH, Graphics::DEFAULT_HEIGHT);

    this->font = TTF_OpenFont("Assets/Consolas.ttf", 12);
}

Graphics::~Graphics()
{
    TTF_CloseFont(this->font);
    SDL_DestroyRenderer(this->renderer);
    SDL_DestroyWindow(this->window);
}

SDL_Texture *Graphics::LoadTexture(char *filename)
{
    SDL_Surface* surface = IMG_Load(filename);
    SDL_Texture* texture = SDL_CreateTextureFromSurface(this->renderer, surface);

    SDL_FreeSurface(surface);
    return texture;
}

void Graphics::SwapBuffer()
{
    SDL_RenderPresent(this->renderer);

    this->Clear();
}

void Graphics::Clear()
{
    SDL_Color oldFGColor = this->fgcolor;
    this->SetFGColor(this->bgcolor);
    SDL_RenderClear(this->renderer);
    this->SetFGColor(oldFGColor);
}

void Graphics::SetFGColor(SDL_Color color)
{
    this->fgcolor = color;
    SDL_SetRenderDrawColor(this->renderer, color.r, color.g, color.b, color.a);
}

void Graphics::SetBGColor(SDL_Color color)
{
    this->bgcolor = color;
}

void Graphics::DrawTexture(SDL_Texture *texture, int x, int y)
{
    SDL_Rect rect;

    rect.x = x;
    rect.y = y;

    SDL_QueryTexture(texture, NULL, NULL, &rect.w, &rect.h);
    SDL_RenderCopy(this->renderer, texture, NULL, &rect);
}

void Graphics::DrawRectangle(int x, int y, int w, int h)
{
    SDL_Rect rect;

    rect.x = x;
    rect.y = y;
    rect.w = w;
    rect.h = h;

    SDL_RenderDrawRect(this->renderer, &rect);
}

void Graphics::DrawRectangle(Vector2D<double> &pos, double angle, double w, double h)
{
    Vector2D<int> rounded = Vector2D<int>((int)round(pos.X()), (int)round(pos.Y()));
    this->DrawRectangle(rounded, angle, w, h);
}

void Graphics::DrawRectangle(Vector2D<int> &pos, double angle, double w, double h)
{
    Vector2D<double> dpos = Vector2D<double>((double)pos.X(), (double)pos.Y());

    Vector2D<double> rotatedHeight = Vector2D<double>(h * cos(angle), h * sin(angle));
    Vector2D<double> rotatedWidth = Vector2D<double>(w * (-sin(angle)), w * (cos(angle)));
    Vector2D<double> rotatedHalfWidth = rotatedWidth * 0.5;

    Vector2D<double> closeLeft = dpos + rotatedHalfWidth;
    Vector2D<double> closeRight = dpos - rotatedHalfWidth;

    Vector2D<double> farLeft = closeLeft + rotatedHeight;
    Vector2D<double> farRight = closeRight + rotatedHeight;

    this->DrawLine(closeLeft, farLeft);
    this->DrawLine(closeRight, farRight);

    this->DrawLine(closeLeft, closeRight);
    this->DrawLine(farLeft, farRight);
}

void Graphics::DrawLine(Vector2D<int> &p1, Vector2D<int> &p2)
{
    SDL_RenderDrawLine(this->renderer, p1.X(), p1.Y(), p2.X(), p2.Y());
}

void Graphics::DrawLine(Vector2D<double> &p1, Vector2D<double> &p2)
{
    Vector2D<int> ip1 = Vector2D<int>((int)round(p1.X()), (int)round(p1.Y()));
    Vector2D<int> ip2 = Vector2D<int>((int)round(p2.X()), (int)round(p2.Y()));

    this->DrawLine(ip1, ip2);
}

Vector2D<double> Graphics::LogicToScreen2D(Vector2D<double> p)
{
    Vector2D<double> result;

    result.x = p.x + (this->Width() / 2.0);
    result.y = (this->Height() / 2.0) - p.y;

    return result;
}

void Graphics::DrawArrow(
    Vector2D<double> &tail, Vector2D<double> &head, const char *fmt, va_list args)
{
    if (fmt == NULL)
        fmt = "";

    this->DrawLine(tail, head);

    Vector2D<double> tipDirection1 = tail - head;
    Vector2D<double> tipDirection2 = tail - head;

    tipDirection1.Rotate(30);
    tipDirection2.Rotate(-30);

    Vector2D<double> tipOffset1 = tipDirection1 * TIP_LENGTH;
    Vector2D<double> tipOffset2 = tipDirection2 * TIP_LENGTH;

    Vector2D<double> tip1 = head + tipOffset1;
    Vector2D<double> tip2 = head + tipOffset2;

    this->DrawLine(head, tip1);
    this->DrawLine(head, tip2);

    Vector2D<double> labelOffset = (head - tail) * TIP_LABEL_LENGTH;
    Vector2D<double> label = head + labelOffset;

    this->PrintStringVAList((int)label.x, (int)label.y, fmt, args);
}

void Graphics::DrawArrow(Vector2D<double> &tail, Vector2D<double> &head, const char *fmt, ...)
{
    if (fmt == NULL)
        fmt = "";

    va_list args;
    va_start(args, fmt);
    this->DrawArrow(tail, head, fmt, args);
    va_end(args);
}

void Graphics::DrawCircle(Vector2D<double> location, double radius)
{
    this->DrawCircle((int)round(location.X()), (int)round(location.Y()), (int)round(radius));
}

void Graphics::DrawCircle(int x, int y, int r)
{
    Vector2D<double> center = Vector2D<double>(x, y);
    Vector2D<double> prev = Vector2D<double>((double)x + (double)r, (double)y);

    double step = M_PI / 50;
    for (double theta = 0; theta <= M_PI * 2.0; theta += step)
    {
        Vector2D<double> translation = Vector2D<double>(cos(theta), sin(theta)) * (double)r;
        Vector2D<double> curr = center + translation;

        Vector2D<int> currInt = Vector2D<int>((int)round(curr.X()), (int)round(curr.Y()));
        Vector2D<int> prevInt = Vector2D<int>((int)round(prev.X()), (int)round(prev.Y()));

        this->DrawLine(prevInt, currInt);

        prev = curr;
    }
}

void Graphics::DrawPie(
    Vector2D<double> location, Vector2D<double> direction, double thetaWidth, double r)
{
    double angle = atan2(direction.Y(), direction.X());
    this->DrawPie(location, thetaWidth, angle, r);
}

void Graphics::DrawPie(Vector2D<double> location, double thetaWidth, double angle, double r)
{
    Vector2D<double> prev = location;

    int low = Graphics::NUM_ARC_STEPS / -2;
    int high = Graphics::NUM_ARC_STEPS / 2;

    for (int step = low; step <= high; step++)
    {
        double deltaTheta = (double)step * thetaWidth / (double)Graphics::NUM_ARC_STEPS;
        double theta = angle + deltaTheta;
        Vector2D<double> translation = Vector2D<double>(cos(theta), sin(theta)) * r;
        Vector2D<double> curr = location + translation;

        Vector2D<int> currInt = Vector2D<int>((int)round(curr.X()), (int)round(curr.Y()));
        Vector2D<int> prevInt = Vector2D<int>((int)round(prev.X()), (int)round(prev.Y()));

        this->DrawLine(prevInt, currInt);

        prev = curr;
    }

    Vector2D<int> finisher1 = Vector2D<int>((int)round(location.X()), (int)round(location.Y()));
    Vector2D<int> finisher2 = Vector2D<int>((int)round(prev.X()), (int)round(prev.Y()));

    this->DrawLine(finisher1, finisher2);
}

void Graphics::PrintString(int x, int y, const char *fmt, ...)
{
    va_list args;

    va_start(args, fmt);
    this->PrintStringVAList(x, y, fmt, args);
    va_end(args);

}

void Graphics::PrintStringVAList(int x, int y, const char *fmt, va_list args)
{
    char formatted[256];
    vsnprintf(formatted, 255, fmt, args);
    SDL_Surface *surface = TTF_RenderText_Solid(this->font, formatted, this->fgcolor);
    SDL_Texture *texture = SDL_CreateTextureFromSurface(this->renderer, surface);

    SDL_Rect unscaledRect;
    unscaledRect.x = x;
    unscaledRect.y = y;
    unscaledRect.w = surface->w;
    unscaledRect.h = surface->h;

    SDL_RenderCopy(this->renderer, texture, NULL, &unscaledRect);
    SDL_DestroyTexture(texture);
    SDL_FreeSurface(surface);
}

void Graphics::ToggleFullScreeen()
{
    if (this->fullscreen)
        SDL_SetWindowFullscreen(this->window, SDL_WINDOW_FULLSCREEN_DESKTOP);
    else
        SDL_SetWindowFullscreen(this->window, 0);

    this->fullscreen = !this->fullscreen;
}
