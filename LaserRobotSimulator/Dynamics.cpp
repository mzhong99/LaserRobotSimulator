#include "Dynamics.hpp"

#define DIFFERENTIAL_DELTA      (1e-8)
#define LIN_INTERP_NPOINTS      (64)

double Dynamics::KineticAtPose(std::vector<double> qPos, std::vector<double> qVel)
{
    std::vector<DHParam> dhParams = m_dhParams;
    std::vector<Transform> transforms;

    for (size_t i = 0; i < qPos.size(); i++)
    {
        dhParams[i].SetQ(qPos[i]);
        transforms.push_back(Transform(dhParams[i]).Inverse());
    }

    double totalKinetic = 0;

    Vector3D<double> prevAngVel = 0;
    Vector3D<double> prevLinVel = 0;

    for (size_t i = 0; i < dhParams.size(); i++)
    {
        /* Joint velocity becomes either a thetaDot or a dispDot, depending on joint type */
        double thetaDot = dhParams[i].Type == JointType::REVOLUTE ? qVel[i] : 0;
        double dispDot = dhParams[i].Type == JointType::PRISMATIC ? qVel[i] : 0;

        Rotation rotPrevToCurr = transforms[i].GetRotation();

        /* Angular velocity computation */
        Vector3D<double> angInner = prevAngVel + Vector3D<double>(0, 0, thetaDot);
        Vector3D<double> angVel = rotPrevToCurr.Rotate(angInner);

        /* Linear velocity computation */
        Vector3D<double> linInner = prevLinVel + Vector3D<double>(0, 0, dispDot);
        Vector3D<double> frameDisplace = transforms[i].GetDisplacement();
        Vector3D<double> rotOffset = Vector3D<double>::Cross(angVel, frameDisplace);
        Vector3D<double> linVel = rotPrevToCurr.Rotate(linInner) + rotOffset;

        /* Center of mass linear velocity computation */
        Vector3D<double> centerOfMass = (frameDisplace * 0.5);
        Vector3D<double> linCenterVel = linVel + Vector3D<double>::Cross(angVel, centerOfMass);

        /* Now, compute the kinetic energy from this specific joint */
        double kinetic = (0.5 * linCenterVel.Length2()) + (0.5 * angVel.Length2());

        /* Accumulate counters. */
        totalKinetic += kinetic;
        prevAngVel = angVel;
        prevLinVel = linVel;
    }

    return totalKinetic;
}

double Dynamics::PotentialAtPose(std::vector<double> qPos)
{
    ForwardKinematics fwdK = ForwardKinematics(m_dhParams);

    for (size_t i = 0; i < qPos.size(); i++)
        fwdK.SetQ(i, qPos[i]);

    fwdK.Recompute();

    std::vector<CoordinateStub> stubs = fwdK.Stubs();
    double totalPotential = 0;

    Vector3D<double> prevPos = 0;
    for (CoordinateStub &stub : stubs)
    {
        Vector3D<double> center = (prevPos + stub.Position) * 0.5;

        /* Assumes all masses are 1.0 */
        double potential = center.Dot(GRAVITY_IN_BASE);
        totalPotential += potential;

        prevPos = stub.Position;
    }

    return totalPotential;
}

double Dynamics::LagrAtPose(std::vector<double> qPos, std::vector<double> qVel)
{
    double kinetic = this->KineticAtPose(qPos, qVel);
    double potential = this->PotentialAtPose(qPos);

    return kinetic - potential;
}

std::vector<double> Dynamics::LagrVsQAtTime(double time)
{
    std::vector<double> qPos, qVel;

    for (JointPath &path : m_jointPaths)
    {
        qPos.push_back(path.QPosAt(time));
        qVel.push_back(path.QVelAt(time));
    }

    double lagrBase = this->LagrAtPose(qPos, qVel);
    std::vector<double> results;

    for (size_t i = 0; i < m_dhParams.size(); i++)
    {
        qPos[i] += DIFFERENTIAL_DELTA;
        double lagrShifted = this->LagrAtPose(qPos, qVel);
        qPos[i] -= DIFFERENTIAL_DELTA;

        results.push_back((lagrShifted - lagrBase) / DIFFERENTIAL_DELTA);
    }

    return results;
}

std::vector<double> Dynamics::LagrVsQPrimeAtTime(double time)
{
    std::vector<double> qPos, qVel;

    for (JointPath &path : m_jointPaths)
    {
        qPos.push_back(path.QPosAt(time));
        qVel.push_back(path.QVelAt(time));
    }

    double lagrBase = this->LagrAtPose(qPos, qVel);
    std::vector<double> results;

    for (size_t i = 0; i < m_dhParams.size(); i++)
    {
        qVel[i] += DIFFERENTIAL_DELTA;
        double lagrShifted = this->LagrAtPose(qPos, qVel);
        qVel[i] -= DIFFERENTIAL_DELTA;

        results.push_back((lagrShifted - lagrBase) / DIFFERENTIAL_DELTA);
    }

    return results;
}

void Dynamics::Precompute()
{
    m_qPosInterpolated.resize(m_dhParams.size());
    m_qVelInterpolated.resize(m_dhParams.size());
    m_torquesInterpolated.resize(m_dhParams.size());

    for (size_t i = 0; i < LIN_INTERP_NPOINTS; i++)
    {
        double time = m_travelTime * (double)i / (double)LIN_INTERP_NPOINTS;
        this->RecomputeAtTime(time);

        for (size_t qIter = 0; qIter < m_dhParams.size(); qIter++)
        {
            m_qPosInterpolated[qIter].AddTime(time, m_qPos[qIter]);
            m_qVelInterpolated[qIter].AddTime(time, m_qVel[qIter]);
            m_torquesInterpolated[qIter].AddTime(time, m_torques[qIter]);
        }

        std::cout << "Computing: " << i << std::endl;
    }
}

void Dynamics::RecomputeAtTime(double time)
{
    std::vector<double> lagrVsQ = this->LagrVsQAtTime(time);

    std::vector<double> lagrVsQPrime = this->LagrVsQPrimeAtTime(time);
    std::vector<double> lagrVsQPrime2 = this->LagrVsQPrimeAtTime(time + DIFFERENTIAL_DELTA);
    std::vector<double> ddtLagrVsQPrime;

    for (size_t i = 0; i < m_dhParams.size(); i++)
        ddtLagrVsQPrime.push_back((lagrVsQPrime2[i] - lagrVsQPrime[i]) / DIFFERENTIAL_DELTA);

    m_torques = std::vector<double>();
    for (size_t i = 0; i < m_dhParams.size(); i++)
        m_torques.push_back(ddtLagrVsQPrime[i] - lagrVsQ[i]);

    m_qPos = std::vector<double>();
    for (size_t i = 0; i < m_dhParams.size(); i++)
        m_qPos.push_back(m_jointPaths[i].QPosAt(time));

    m_qVel = std::vector<double>();
    for (size_t i = 0; i < m_dhParams.size(); i++)
        m_qVel.push_back(m_jointPaths[i].QVelAt(time));
}

std::vector<double> Dynamics::TorquesAtTime(double time)
{
    std::vector<double> torques;

    for (LinearInterpolator &interpolator : m_torquesInterpolated)
        torques.push_back(interpolator.Interpolate(time));

    return torques;
}

std::vector<double> Dynamics::QPosAtTime(double time)
{
    std::vector<double> qPos;

    for (LinearInterpolator &interpolator : m_qPosInterpolated)
        qPos.push_back(interpolator.Interpolate(time));

    return qPos;
}

std::vector<double> Dynamics::QVelAtTime(double time)
{
    std::vector<double> qVel;

    for (LinearInterpolator &interpolator : m_qVelInterpolated)
        qVel.push_back(interpolator.Interpolate(time));

    return qVel;
}
