#pragma once

#include "Vector3D.hpp"
#include "Rotation.hpp"
#include "Transform.hpp"

struct CoordinateStub
{
    Vector3D<double> Position;
    Rotation Orientation;

    Transform GetTransform() { return Transform(this->Orientation, this->Position); }

    CoordinateStub(): Position(0) {};
    CoordinateStub(Vector3D<double> position, Rotation orientation): 
        Position(position), Orientation(orientation) {};
};
