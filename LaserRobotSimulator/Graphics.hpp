#pragma once

#include <SDL.h>
#include <SDL_image.h>
#include <SDL_ttf.h>

#include <string>

#include "Vector2D.hpp"

class Graphics
{
private:
    static const int NUM_ARC_STEPS = 120;

    SDL_Window *window;
    SDL_Renderer *renderer;
    TTF_Font *font;

    SDL_Color fgcolor;
    SDL_Color bgcolor;

    bool fullscreen = false;

    void PrintStringVAList(int x, int y, const char *fmt, va_list args);

public:
    Graphics();
    ~Graphics();

    static const int DEFAULT_WIDTH = 1280;
    static const int DEFAULT_HEIGHT = 720;

    static const SDL_Color COLOR_BLACK;
    static const SDL_Color COLOR_WHITE;
    static const SDL_Color COLOR_RED;
    static const SDL_Color COLOR_GREEN;
    static const SDL_Color COLOR_BLUE;
    static const SDL_Color COLOR_GRAY;

    SDL_Texture *LoadTexture(char *filename);

    void SwapBuffer();
    void Clear();

    void SetFGColor(SDL_Color color);
    void SetBGColor(SDL_Color color);

    void DrawTexture(SDL_Texture *texture, int x, int y);

    void DrawRectangle(int x, int y, int w, int h);
    
    void DrawRectangle(Vector2D<double> &pos, double angle, double w, double h);
    void DrawRectangle(Vector2D<int> &pos, double angle, double w, double h);

    void DrawCircle(Vector2D<double> location, double r);
    void DrawCircle(int x, int y, int r);

    void DrawPie(Vector2D<double> location, Vector2D<double> facing, double thetaWidth, double r);
    void DrawPie(Vector2D<double> location, double thetaWidth, double angle, double r);

    void DrawLine(Vector2D<int> &p1, Vector2D<int> &p2);
    void DrawLine(Vector2D<double> &p1, Vector2D<double> &p2);

    void DrawArrow(Vector2D<double> &tail, Vector2D<double> &head, const char *fmt = "", ...);

    void PrintString(int x, int y, const char *fmt, ...);

    uint32_t Width() { return Graphics::DEFAULT_WIDTH; }
    uint32_t Height() { return Graphics::DEFAULT_HEIGHT; }

    Vector2D<double> LogicToScreen2D(Vector2D<double> p);

    void ToggleFullScreeen();
};

