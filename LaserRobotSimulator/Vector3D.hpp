#ifndef __VECTOR_HPP__
#define __VECTOR_HPP__

#include <cmath>
#include <iostream>
#include <fstream>

template <typename T>
class Vector3D
{
public:
    T x, y, z;

    Vector3D(): x(T(0)), y(T(0)), z(T(0)) {}
    Vector3D(T val): x(val), y(val), z(val) {}

    Vector3D(T x, T y, T z): x(x), y(y), z(z) {}

    T Dot(const Vector3D<T> &rhs) const
    { return (x * rhs.x) + (y * rhs.y) + (z * rhs.z); }

    Vector3D<T> Normalized() const
    {
        Vector3D<T> result = *this;
        T mag2 = this->Length2();

        if (mag2 > 0)
        {
            T invmag = 1.0 / sqrt(mag2);
            result *= invmag;
        }

        return result;
    }

    bool operator==(const Vector3D<T> &rhs)
    {
        return fabs(rhs.x - this->x) < 1e-1 &&
            fabs(rhs.y - this->y) < 1e-1 &&
            fabs(rhs.z - this->z) < 1e-1;
    }

    Vector3D<T> ElementMult(const Vector3D<T> &rhs)
    { return Vector3D<T>(this->x * rhs.x, this->y * rhs.y, this->z * rhs.z); }

    Vector3D<T> operator*(const T &val) const
    { return Vector3D<T>(this->x * val, this->y * val, this->z * val); }

    Vector3D<T> operator+(const Vector3D<T>& rhs) const
    { return Vector3D<T>(this->x + rhs.x, this->y + rhs.y, this->z + rhs.z); }

    Vector3D<T> operator-(const Vector3D<T>& rhs) const
    { return Vector3D<T>(this->x - rhs.x, this->y - rhs.y, this->z - rhs.z); }

    Vector3D<T> &operator+=(const Vector3D<T> &rhs)
    {
        this->x += rhs.x;
        this->y += rhs.y;
        this->z += rhs.z;

        return *this;
    }

    Vector3D<T> &operator-=(const Vector3D<T> &rhs)
    {
        this->x -= rhs.x;
        this->y -= rhs.y;
        this->z -= rhs.z;

        return *this;
    }

    Vector3D<T> &operator*=(const T &val)
    {
        this->x *= val;
        this->y *= val;
        this->z *= val;

        return *this;
    }

    static Vector3D<T> Cross(const Vector3D<T> &lhs, const Vector3D<T> &rhs)
    {
        Vector3D<T> result;

        result.x = (lhs.y * rhs.z) - (lhs.z * rhs.y);
        result.y = (lhs.z * rhs.x) - (lhs.x * rhs.z);
        result.z = (lhs.x * rhs.y) - (lhs.y * rhs.x);

        return result;
    }

    Vector3D<T> operator-() const 
    { return Vector3D<T>(-this->x, -this->y, -this->z); }

    T Length2() const { return this->Dot(*this); }
    T Length() const { return sqrt(this->Length2()); }

    friend std::ostream &operator<<(std::ostream &os, const Vector3D<T> &vec)
    { 
        os << "[" << +vec.x << " " << +vec.y << " " << +vec.z << "]"; 
        return os; 
    }
};

class Ray
{
public:
    const Vector3D<double> origin;
    const Vector3D<double> direction;

    Ray(): origin(Vector3D<double>(0)), direction(Vector3D<double>(0)) {}
    Ray(const Vector3D<double> &origin, const Vector3D<double> &direction):
        origin(origin), direction(direction.Normalized()) {}
};

#endif