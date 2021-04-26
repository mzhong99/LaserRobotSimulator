#pragma once

#include <vector>
#include "Vector3D.hpp"

#include "Matrix.hpp"
#include "DHParam.hpp"
#include "CoordinateStub.hpp"
#include "Transform.hpp"

#include "RobotFwd.hpp"

class ForwardKinematics
{
private:
    void RecomputeCoordinateStubs();
    void RecomputeJacobian();
    void RecomputeFwdDK();

    std::vector<double> m_qPrime;
    std::vector<DHParam> m_dhParams;
    std::vector<CoordinateStub> m_stubs;

    Matrix m_jacobian;
    Vector3D<double> m_eeLinPos;
    Vector3D<double> m_eeAngPos;

    Vector3D<double> m_eeLinVel;
    Vector3D<double> m_eeAngVel;

public:
    ForwardKinematics() = default;
    ForwardKinematics(std::vector<DHParam> params);

    void Recompute();

    double GetQ(size_t idx) { return m_dhParams[idx].GetQ(); }
    void SetQ(size_t idx, double val) { m_dhParams[idx].SetQ(val); }
    void AccQ(size_t idx, double delta) { m_dhParams[idx].SetQ(m_dhParams[idx].GetQ() + delta); }

    void SetQPrime(size_t idx, double value) { m_qPrime[idx] = value; }
    void AccQPrime(size_t idx, double delta) { m_qPrime[idx] += delta; }

    Transform TransformEEToBase() { return m_stubs.back().GetTransform(); }

    Vector3D<double> EELinPos() { return m_eeLinPos; }
    Vector3D<double> EEAngPos() { return m_eeAngPos; }

    Vector3D<double> EELinVel() { return m_eeLinVel; }
    Vector3D<double> EEAngVel() { return m_eeAngVel; }

    Matrix &Jacobian() { return m_jacobian; }
    std::vector<CoordinateStub> &Stubs() { return m_stubs; }
    std::vector<double> &QPrime() { return m_qPrime; }
    std::vector<DHParam> &DHParams() { return m_dhParams; }
};

