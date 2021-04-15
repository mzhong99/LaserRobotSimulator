#pragma once
#include <cmath>
#include <SDL.h>

template <typename T> 
class Vector2D 
{
public:
    T x, y;

    Vector2D() :x(0), y(0) {}
    Vector2D(T x, T y) : x(x), y(y) {}
    Vector2D(const Vector2D &v) : x(v.x), y(v.y) {}

    T X() { return this->x; }
    T Y() { return this->y; }

    Vector2D &operator=(const Vector2D &v) 
    {
        x = v.x;
        y = v.y;
        return *this;
    }

    Vector2D operator+(Vector2D &v) { return Vector2D(x + v.x, y + v.y); }
    Vector2D operator-(Vector2D &v) { return Vector2D(x - v.x, y - v.y); }

    Vector2D &operator+=(Vector2D &v) 
    {
        x += v.x;
        y += v.y;
        return *this;
    }

    Vector2D &operator-=(Vector2D &v) 
    {
        x -= v.x;
        y -= v.y;
        return *this;
    }

    Vector2D operator+(double s) { return Vector2D(x + s, y + s); }
    Vector2D operator-(double s) { return Vector2D(x - s, y - s); }
    Vector2D operator*(double s) { return Vector2D(x * s, y * s); }
    Vector2D operator/(double s) { return Vector2D(x / s, y / s); }

    Vector2D &operator+=(double s) 
    {
        x += s;
        y += s;
        return *this;
    }

    Vector2D &operator-=(double s) 
    {
        x -= s;
        y -= s;
        return *this;
    }

    Vector2D &operator*=(double s) 
    {
        x *= s;
        y *= s;
        return *this;
    }

    Vector2D &operator/=(double s) 
    {
        x /= s;
        y /= s;
        return *this;
    }

    void Rotate(double deg) 
    {
        double theta = deg / 180.0 * M_PI;

        double c = cos(theta);
        double s = sin(theta);

        double tx = x * c - y * s;
        double ty = x * s + y * c;

        x = tx;
        y = ty;
    }

    Vector2D &Normalize() 
    {
        if (this->Length() == 0) 
            return *this;

        *this *= (1.0 / this->Length());
        return *this;
    }

    float Dist(Vector2D v) const 
    {
        Vector2D d(v.x - x, v.y - y);
        return d.Length();
    }

    double Length() const { return std::sqrt(x * x + y * y); }

    void Truncate(double length) 
    {
        double angle = atan2f(y, x);
        x = length * cos(angle);
        y = length * sin(angle);
    }

    static float Dot(Vector2D v1, Vector2D v2) {
        return v1.x * v2.x + v1.y * v2.y;
    }
    static float Cross(Vector2D v1, Vector2D v2) {
        return (v1.x * v2.y) - (v1.y * v2.x);
    }

};
