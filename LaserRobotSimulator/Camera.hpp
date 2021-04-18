#pragma once

#include "Transform.hpp"
#include "Rotation.hpp"

#include "Vector3D.hpp"
#include "Vector2D.hpp"

#include "Simulator.hpp"

#include <cmath>
#include <SDL.h>

#define DEFAULT_ISO_ZOOM_FACTOR     (72.0)
#define ISO_ZOOM_VELOCITY           (15.0)

#define CAMERA_SPEED                (2.0)

class Camera
{
private:
    Vector3D<double> m_posWorldCoords;

    Rotation m_rotation;
    Transform m_worldToBase;
    Transform m_baseToWorld;

    double m_aboutCameraY;
    double m_aboutCameraX;

    double m_isoZoomFactor;
    Vector2D<double> m_isoScreenOffset;

    bool m_usePerspective;

    Vector2D<double> PerspectiveWorldToScreen(Vector3D<double> pointInWorld);
    Vector2D<double> IsometricWorldToScreen(Vector3D<double> pointInWorld);

public:
    Camera();

    void AccumulateXRotation(double delta);
    void AccumulateYRotation(double delta) { m_aboutCameraY += delta; }

    void AccumulateForwards(double direction);
    void AccumulateSideways(double direction);
    void AccumulateUpDown(double direction) 
    { m_posWorldCoords.y += Simulator::App().DetaTimeSeconds() * CAMERA_SPEED * direction; }

    Vector2D<double> WorldToScreen(Vector3D<double> pointInWorld);
    void DrawArrow(
        Vector3D<double> &tailInWorld, Vector3D<double> &headInWorld, const char *fmt, ...);

    void RefreshView();

    void ZoomIn() 
    { m_isoZoomFactor += ISO_ZOOM_VELOCITY * Simulator::App().DetaTimeSeconds(); }

    void ZoomOut()
    { m_isoZoomFactor -= ISO_ZOOM_VELOCITY * Simulator::App().DetaTimeSeconds(); }

    void AccumulateScreenOffset(Vector2D<int> delta)
    { this->AccumulateScreenOffset(Vector2D<double>(delta.x, delta.y)); }

    void AccumulateScreenOffset(Vector2D<double> delta)
    { this->m_isoScreenOffset -= delta; }

    void ToggleUsePerspective() { m_usePerspective = !m_usePerspective; }
};

