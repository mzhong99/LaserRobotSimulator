#include "JointPath.hpp"

#include <cmath>

void JointPath::Precompute()
{
    std::vector<double> raw =
    {
        1 * pow(m_deltaT, 3),  1 * pow(m_deltaT, 4),  1 * pow(m_deltaT, 5), m_qEnd - m_qBegin,
        3 * pow(m_deltaT, 2),  4 * pow(m_deltaT, 3),  5 * pow(m_deltaT, 4), 0,
        6 * pow(m_deltaT, 1), 12 * pow(m_deltaT, 2), 20 * pow(m_deltaT, 3), 0
    };

    Matrix constFinder = Matrix(3, 4, raw);
    constFinder.GaussJordanEliminate();

    m_aConst = constFinder.GetCol(3).Flatten();
}

double JointPath::QPosAt(double time)
{
    return m_qBegin
        + (m_aConst[0] * pow(time, 3))
        + (m_aConst[1] * pow(time, 4))
        + (m_aConst[2] * pow(time, 5));
}

double JointPath::QVelAt(double time)
{
    return 3 * m_aConst[0] * pow(time, 2)
        + (4 * m_aConst[1] * pow(time, 3))
        + (5 * m_aConst[2] * pow(time, 4));
}
