#include "Camera.hpp"
#include "Simulator.hpp"


Camera::Camera()
{
    m_posWorldCoords = Vector3D<double>(0);
    m_aboutCameraX = 0;
    m_aboutCameraY = 0;

    m_usePerspective = true;

    m_isoZoomFactor = DEFAULT_ISO_ZOOM_FACTOR;
    m_perspectiveZoomFactor = DEFAULT_PERSPECTIVE_ZOOM_FACTOR;
}

void Camera::AccumulateXRotation(double delta)
{
    m_aboutCameraX += delta;

    if (m_aboutCameraX > M_PI / 2.0)
        m_aboutCameraX = M_PI / 2.0;

    if (m_aboutCameraX < -M_PI / 2.0)
        m_aboutCameraX = -M_PI / 2.0;
}

void Camera::RefreshView()
{
    m_aboutCameraX = fmod(m_aboutCameraX, 2 * M_PI);
    m_aboutCameraY = fmod(m_aboutCameraY, 2 * M_PI);

    Rotation aboutX = Rotation::CreateAboutX(m_aboutCameraX);
    Rotation aboutY = Rotation::CreateAboutY(m_aboutCameraY);

    m_rotation = Rotation::Multiply(aboutX, aboutY);

    Transform shifted = Transform(Rotation(), m_posWorldCoords * -1.0);
    Transform rotated = Transform(m_rotation, 0);

    m_worldToBase = Transform::Multiply(rotated, shifted);
    m_baseToWorld = m_worldToBase.Inverse();
}

void Camera::AccumulateForwards(double direction)
{
    double forwardsX = sin(m_aboutCameraY);
    double forwardsZ = cos(m_aboutCameraY);

    Vector3D<double> unit = Vector3D<double>(forwardsX, 0, forwardsZ);
    m_posWorldCoords += unit * Simulator::App().DetaTimeSeconds() * (CAMERA_SPEED * direction);
}

void Camera::AccumulateSideways(double direction)
{
    double sidewaysX = sin(m_aboutCameraY + (M_PI / 2.0));
    double sidewaysZ = cos(m_aboutCameraY + (M_PI / 2.0));

    Vector3D<double> unit = Vector3D<double>(sidewaysX, 0, sidewaysZ);
    m_posWorldCoords += unit * Simulator::App().DetaTimeSeconds() * (CAMERA_SPEED * direction);
}

Vector2D<double> Camera::PerspectiveWorldToScreen(Vector3D<double> pointInWorld)
{
    Vector3D<double> pointInBase = m_worldToBase.TransformPoint(pointInWorld);

    if (pointInBase.z < -0.9)
        return Vector2D<double>(1e6, 1e6);

    double zw = fabs(pointInBase.z);

    double W = Simulator::Graphics().Width();
    double H = Simulator::Graphics().Height();
    double A = W / H;

    double xw = pointInBase.x;
    double yw = pointInBase.y;

    double thetaFOV = M_PI * (pow(75.0, m_perspectiveZoomFactor) / 180.0);
    double tangent = tan(thetaFOV / 2.0);

    double xv = xw / zw;
    double yv = yw / zw;

    double xi = 0.5 * (W * ((xv / (A * tangent)) + 1.0) - 1.0);
    double yi = 0.5 * (H * (1.0 - (yv / tangent)) - 1.0);

    return Vector2D<double>(xi, yi);
}

Vector2D<double> Camera::IsometricWorldToScreen(Vector3D<double> pointInWorld)
{
    Vector3D<double> pointInBase = m_worldToBase.TransformPoint(pointInWorld);

    Vector2D<double> point2D = Vector2D<double>(pointInBase.x, pointInBase.y);
    Vector2D<double> scPoint2D = Simulator::Graphics().LogicToScreen2D(point2D * m_isoZoomFactor);
    return scPoint2D - m_isoScreenOffset;
}

Vector2D<double> Camera::WorldToScreen(Vector3D<double> pointInWorld)
{
    if (m_usePerspective)
        return this->PerspectiveWorldToScreen(pointInWorld);

    return this->IsometricWorldToScreen(pointInWorld);
}

void Camera::DrawArrow(
    Vector3D<double> tailInWorld, Vector3D<double> headInWorld, const char *fmt, ...)
{
    Vector3D<double> tailInBase = m_worldToBase.TransformPoint(tailInWorld);
    Vector3D<double> headInBase = m_worldToBase.TransformPoint(headInWorld);

    if ((tailInBase.z < 0.0 || headInBase.z < 0.0) && this->m_usePerspective)
        return;

    Vector2D<double> tailInScreen = this->WorldToScreen(tailInWorld);
    Vector2D<double> headInScreen = this->WorldToScreen(headInWorld);

    va_list args;
    va_start(args, fmt);
    Simulator::Graphics().DrawArrow(tailInScreen, headInScreen, fmt, args);
    va_end(args);
}


void Camera::DrawArrowFromOffset(
    Vector3D<double> origin, Vector3D<double> direction, const char *fmt, ...)
{
    Vector3D<double> head = origin + direction;

    Vector3D<double> tailInBase = m_worldToBase.TransformPoint(origin);
    Vector3D<double> headInBase = m_worldToBase.TransformPoint(head);

    if ((tailInBase.z < 0.0 || headInBase.z < 0.0) && this->m_usePerspective)
        return;

    Vector2D<double> tailInScreen = this->WorldToScreen(origin);
    Vector2D<double> headInScreen = this->WorldToScreen(head);

    va_list args;
    va_start(args, fmt);
    Simulator::Graphics().DrawArrow(tailInScreen, headInScreen, fmt, args);
    va_end(args);
}
