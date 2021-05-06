#include "Robot.hpp"
#include <cmath> 
#include <SDL.h>

#include <iomanip>

#define DEFAULT_DISTANCE        (0.0)
#define DIFFERENTIAL_DELTA      (1e-10)

/**************************************************************************************************/
/* Model ---------------------------------------------------------------------------------------- */
/**************************************************************************************************/
Robot::Robot()
{
    m_simulationMode = SimulationMode::VIEW_ALL;

    m_dhParams.push_back(DHParam::CreatePrismatic(M_PI / 2.0, M_PI / 2.0, 0));
    m_dhParams.push_back(DHParam::CreatePrismatic(M_PI / 2.0, M_PI / 2.0, 0));
    m_dhParams.push_back(DHParam::CreatePrismatic(M_PI / 2.0, M_PI / 2.0, 0));
    m_dhParams.push_back(DHParam::CreateRevolute(M_PI / 2.0, 0, 0));
    m_dhParams.push_back(DHParam::CreateRevolute(M_PI / 2.0, 0, 0));
    m_dhParams.push_back(DHParam::CreateRevolute(M_PI / 2.0, 0, 0));

    m_fwdK = ForwardKinematics(m_dhParams);
    for (size_t qIter = 0; qIter < this->NumJoints(); qIter++)
        m_fwdK.SetQPrime(qIter, 1.2);

    m_invK = new InverseKinematics(m_dhParams);
    m_statics = Statics(&m_fwdK);
}


void Robot::Recompute()
{
    m_fwdK.Recompute();
    m_statics.Recompute();
}

