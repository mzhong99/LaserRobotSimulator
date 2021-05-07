#pragma once

#include "Matrix.hpp"
#include <vector>

class JointPath
{
private:
    double m_qBegin;
    double m_qEnd;
    double m_deltaT;

    std::vector<double> m_aConst;

    void Precompute();

public:
    JointPath() = default;

    JointPath(double qBegin, double qEnd, double travelTime):
        m_qBegin(qBegin), m_qEnd(qEnd), m_deltaT(travelTime) { this->Precompute(); }

    double QPosAt(double time);
    double QVelAt(double time);

    std::vector<double> GetConstants() { return m_aConst; }
};

