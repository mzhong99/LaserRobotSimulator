#include "InverseKinematics.hpp"
#include "Robot.hpp"

#define NEWTON_STEP_DELTA   (5e-3)
#define ACCEPTABLE_ERROR    (1e-2)

std::vector<double> InverseKinematics::EEVelToJointVel(Matrix eeVel)
{
    Matrix jacobianAug1 = Matrix::LRJoin(m_fwdK.Jacobian(), eeVel);
    Matrix jacobianAug2 = jacobianAug1;

    /* First, assume all free variables are zeros. Compute the joint deltas once. */
    std::vector<double> zeros(m_dhParams.size(), 0.0);
    std::vector<bool> isFreeVar(m_dhParams.size(), false);
    std::vector<double> naive = jacobianAug1.RREFWithFreeVariables(zeros, isFreeVar);

    /* Next, assume the free variables are all the average of the basic variables. Recompute. */
    double fillVal = std::accumulate(naive.begin(), naive.end(), 0.0) / (double)m_dhParams.size();
    std::fill(isFreeVar.begin(), isFreeVar.end(), false);
    std::vector<double> avgFreeVars = std::vector<double>(m_dhParams.size(), fillVal);

    /* The answer is returned after a single average. */
    return jacobianAug2.RREFWithFreeVariables(avgFreeVars, isFreeVar);
}

void InverseKinematics::CopyInputs()
{
    std::lock_guard<std::mutex> guard(m_inputLock);

    m_eeTargetLinPos = m_iEELinPos;
    m_eeTargetAngPos = m_iEEAngPos;
    m_eeVel = m_iEEVel;
}

void InverseKinematics::RecomputeDistanceCorrection()
{
    m_linError = (m_eeTargetLinPos - m_fwdK.EELinPos());
    m_angError = (m_eeTargetAngPos - m_fwdK.EEAngPos());
    m_error = m_linError.Length() + m_angError.Length();
}

void InverseKinematics::NewtonApproximateJoints()
{
    Vector3D<double> linDir = m_linError.Normalized();
    Vector3D<double> angDir = m_angError.Normalized();

    std::vector<double> eeVRaw = { linDir.x, linDir.y, linDir.z, angDir.x, angDir.y, angDir.z };
    Matrix eeVCol = Matrix::CreateCol(eeVRaw);
    std::vector<double> qPrime = this->EEVelToJointVel(eeVCol);

    for (size_t qIter = 0; qIter < m_dhParams.size(); qIter++)
        m_fwdK.AccQ(qIter, qPrime[qIter] * NEWTON_STEP_DELTA * sqrt(m_error));

    m_fwdK.Recompute();
}

void InverseKinematics::ReportAnswer()
{
    std::lock_guard<std::mutex> oguard(m_outputLock);
    m_oPose = m_fwdK.DHParams();
    m_oQPrime = m_qPrime;

    m_oEELinPos = m_fwdK.EELinPos();
    m_oEEAngPos = m_fwdK.EEAngPos();
    m_oTransformEEtoBase = m_fwdK.TransformEEToBase();
    m_oStubs = m_fwdK.Stubs();
    
    std::lock_guard<std::mutex> iguard(m_inputLock);
    m_oEEVel = m_iEEVel;
}

void InverseKinematics::TF_ComputeJointPosition()
{
    for (;;)
    {
        if (m_killswitch)
            break;

        this->CopyInputs();
        this->RecomputeDistanceCorrection();
        this->NewtonApproximateJoints();
        this->RecomputeInvDK();
        this->ReportAnswer();
    }
}

InverseKinematics::InverseKinematics(std::vector<DHParam> params)
{
    m_killswitch = false;
    m_dhParams = params;

    for (DHParam &param : m_dhParams)
        param.SetQ(0);

    m_fwdK = ForwardKinematics(params);
    m_fwdK.Recompute();

    m_oStubs.resize(m_dhParams.size());
    m_oPose.resize(m_dhParams.size());
    m_oQPrime.resize(m_dhParams.size());

    m_iEELinPos = m_fwdK.EELinPos();
    m_iEEAngPos = m_fwdK.EEAngPos();

    m_eeTargetLinPos = m_fwdK.EELinPos();
    m_eeTargetAngPos = m_fwdK.EEAngPos();

    m_worker = std::thread(&InverseKinematics::TF_ComputeJointPosition, this);
}

InverseKinematics::~InverseKinematics()
{
    m_killswitch = true;
    m_worker.join();
}

void InverseKinematics::AccTargetEEPos(Vector3D<double> eeLinDelta, Vector3D<double> eeAngDelta)
{
    std::lock_guard<std::mutex> guard(m_inputLock);

    m_iEELinPos += eeLinDelta;
    m_iEEAngPos += eeAngDelta;
    m_inputChanged = true;
}

void InverseKinematics::AccTargetEELinVel(double deltaMag, double deltaAngX, double deltaAngY)
{
    std::lock_guard<std::mutex> guard(m_inputLock);

    m_iEEVel.Linear().AccumulateMagnitude(deltaMag);
    m_iEEVel.Linear().AccumulateXAngle(deltaAngX);
    m_iEEVel.Linear().AccumulateYAngle(deltaAngY);
}

void InverseKinematics::AccTargetEEAngVel(double deltaMag, double deltaAngX, double deltaAngY)
{
    std::lock_guard<std::mutex> guard(m_inputLock);

    m_iEEVel.Angular().AccumulateMagnitude(deltaMag);
    m_iEEVel.Angular().AccumulateXAngle(deltaAngX);
    m_iEEVel.Angular().AccumulateYAngle(deltaAngY);
}

std::vector<CoordinateStub> InverseKinematics::GetStubs()
{
    std::lock_guard<std::mutex> guard(m_outputLock);
    return m_oStubs;
}

std::vector<DHParam> InverseKinematics::GetPose()
{
    std::lock_guard<std::mutex> guard(m_outputLock);
    return m_oPose;
}

std::vector<double> InverseKinematics::GetQPrime()
{
    std::lock_guard<std::mutex> guard(m_outputLock);
    return m_oQPrime;
}

Vector3D<double> InverseKinematics::EELinPos()
{
    std::lock_guard<std::mutex> guard(m_outputLock);
    return m_oEELinPos;
}

Vector3D<double> InverseKinematics::EEAngPos()
{
    std::lock_guard<std::mutex> guard(m_outputLock);
    return m_oEEAngPos;
}

Vector3D<double> InverseKinematics::EELinVel()
{
    std::lock_guard<std::mutex> guard(m_outputLock);
    return m_oEEVel.Linear().Cartesian();
}

Vector3D<double> InverseKinematics::EEAngVel()
{
    std::lock_guard<std::mutex> guard(m_outputLock);
    return m_oEEVel.Angular().Cartesian();
}

Transform InverseKinematics::TransformEEToBase()
{
    std::lock_guard<std::mutex> guard(m_outputLock);
    return m_oTransformEEtoBase;
}
