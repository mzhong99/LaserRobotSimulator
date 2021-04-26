#include "ForwardKinematics.hpp"
#include "Robot.hpp"

void ForwardKinematics::RecomputeCoordinateStubs()
{
    Transform cumulative = Transform();
    m_eeAngPos = Vector3D<double>(0);

    for (size_t i = 0; i < m_dhParams.size(); i++)
    {
        DHParam dhparam = m_dhParams[i];
        Transform transform = Transform(dhparam);

        /* Compute theta angular position BEFORE shifting by rotation */
        Rotation rotBeforeTransform = cumulative.GetRotation();
        Vector3D<double> thetaJointAngle = Vector3D<double>(0, 0, dhparam.Theta);
        Vector3D<double> thetaJointAngleInBase = rotBeforeTransform.Rotate(thetaJointAngle);
        m_eeAngPos += thetaJointAngleInBase;
        
        /* Perform a right-multiplication since cumulative contains all but the latest */
        cumulative = Transform::Multiply(cumulative, transform);

        Vector3D<double> position = cumulative.GetDisplacement();
        Rotation rotation = cumulative.GetRotation();

        /* Compute alpha angular position AFTER shifting by rotation */
        Vector3D<double> alphaJointAngle = Vector3D<double>(dhparam.Alpha, 0, 0);
        Vector3D<double> alphaJointAngleInBase = rotation.Rotate(alphaJointAngle);
        m_eeAngPos += alphaJointAngleInBase;

        CoordinateStub stub = CoordinateStub(position, rotation);
        m_stubs[i] = stub;
    }

    m_eeLinPos = m_stubs.back().Position;
}

void ForwardKinematics::RecomputeJacobian()
{
    Vector3D<double> prevZHat = Vector3D<double>(0, 0, 1);
    Vector3D<double> prevToEE = this->m_stubs.back().Position;

    for (size_t qIter = 0; qIter < this->m_dhParams.size(); qIter++)
    {
        Vector3D<double> linearV = Vector3D<double>(0);
        Vector3D<double> angularV = Vector3D<double>(0);

        switch (this->m_dhParams[qIter].Type)
        {
            case JointType::PRISMATIC:
                linearV = prevZHat;
                angularV = Vector3D<double>(0);
                break;

            case JointType::REVOLUTE:
                linearV = Vector3D<double>::Cross(prevZHat, prevToEE);
                angularV = prevZHat;
                break;

            default:
                break;
        }

        prevZHat = this->m_stubs[qIter].Orientation.GetColumn(2);
        prevToEE = m_eeLinPos - this->m_stubs[qIter].Position;

        this->m_jacobian.At(0, qIter) = linearV.x;
        this->m_jacobian.At(1, qIter) = linearV.y;
        this->m_jacobian.At(2, qIter) = linearV.z;

        this->m_jacobian.At(3, qIter) = angularV.x;
        this->m_jacobian.At(4, qIter) = angularV.y;
        this->m_jacobian.At(5, qIter) = angularV.z;
    }
}

void ForwardKinematics::RecomputeFwdDK()
{
    Matrix qPrimeCol = Matrix::CreateCol(m_qPrime);
    Matrix eeCol = Matrix::Multiply(m_jacobian, qPrimeCol);

    m_eeLinVel = Vector3D<double>(eeCol.At(0, 0), eeCol.At(1, 0), eeCol.At(2, 0));
    m_eeAngVel = Vector3D<double>(eeCol.At(3, 0), eeCol.At(4, 0), eeCol.At(5, 0));
}

ForwardKinematics::ForwardKinematics(std::vector<DHParam> params)
{
    m_dhParams = params;

    m_stubs.resize(m_dhParams.size());
    m_qPrime.resize(m_dhParams.size());

    m_jacobian = Matrix(6, m_dhParams.size());

    this->Recompute();
}

void ForwardKinematics::Recompute()
{
    this->RecomputeCoordinateStubs();
    this->RecomputeJacobian();

    this->RecomputeFwdDK();
}
