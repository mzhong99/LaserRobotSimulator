#include "Robot.hpp"
#include <cmath> 
#include <SDL.h>

#include <iomanip>

#define DEFAULT_DISTANCE        (0.0)

// #define DEFAULT_ZOOM_FACTOR     (3.0)
// #define ZOOM_VELOCITY           (3.0)

#define DIFFERENTIAL_DELTA      (1e-10)

DHParam DHParam::CreateRevolute(double alpha, double a, double d)
{
    DHParam param;

    param.Alpha = alpha;
    param.A = a;
    param.D = d;
    param.Theta = 1.0;

    param.Type = JointType::REVOLUTE;

    return param;
}

DHParam DHParam::CreatePrismatic(double theta, double alpha, double a)
{
    DHParam param;

    param.Theta = theta;
    param.Alpha = alpha;
    param.A = a;
    param.D = 3.0;

    param.Type = JointType::PRISMATIC;

    return param;
}

double DHParam::GetQ() 
{
    switch (this->Type)
    {
        case JointType::PRISMATIC:
            return this->D;
        case JointType::REVOLUTE:
            return this->Theta;
    }

    return 0;
}

void DHParam::SetQ(double val)
{
    switch (this->Type)
    {
        case JointType::PRISMATIC:
            this->D = val;
            break;

        case JointType::REVOLUTE:
            this->Theta = val;
            break;

        default:
            break;
    }
}

/**************************************************************************************************/
/* Model ---------------------------------------------------------------------------------------- */
/**************************************************************************************************/
void Robot::AccumulateEndEffectorVelocity(double deltaE) 
{
    if (this->m_idx < this->m_manualEndEffectorVelocity.size())
        this->m_manualEndEffectorVelocity[this->m_idx] += deltaE;
}

Robot::Robot()
{
    m_dhParams.push_back(DHParam::CreatePrismatic(M_PI / 2.0, M_PI / 2.0, 0));
    m_dhParams.push_back(DHParam::CreateRevolute(M_PI / 2.0, 0, 0));
    m_dhParams.push_back(DHParam::CreatePrismatic(M_PI / 2.0, M_PI / 2.0, 0));
    m_dhParams.push_back(DHParam::CreateRevolute(M_PI / 2.0, 0, 0));
    m_dhParams.push_back(DHParam::CreatePrismatic(M_PI / 2.0, M_PI / 2.0, 0));
    m_dhParams.push_back(DHParam::CreateRevolute(M_PI / 2.0, 0, 0));
    m_dhParams.push_back(DHParam::CreatePrismatic(0, 0, 0));

    m_coordinateStubs.resize(m_dhParams.size());
    m_manualJointSpeeds.resize(m_dhParams.size(), 1.2);
    m_jointForceMoments.resize(m_dhParams.size());

    m_jacobian = Matrix(6, m_dhParams.size());
    m_manualEndEffectorVelocity.resize(m_jacobian.Rows());
    m_isFreeVariable.resize(m_dhParams.size());

    m_manualEndEffectorVelocity = { 1.0, 2.0, 3.0, -1.0, -2.0, -3.0 };
}

void Robot::RecomputeCoordinateStubs()
{
    Transform cumulative = Transform();

    for (size_t i = 0; i < m_dhParams.size(); i++)
    {
        DHParam dhparam = m_dhParams[i];
        Transform transform = Transform(dhparam.Theta, dhparam.Alpha, dhparam.A, dhparam.D);
        
        /* Perform a right-multiplication since cumulative contains all but the latest */
        cumulative = Transform::Multiply(cumulative, transform);

        Vector3D<double> position = cumulative.GetDisplacement();
        Rotation rotation = cumulative.GetRotation();

        CoordinateStub stub = CoordinateStub(position, rotation);
        m_coordinateStubs[i] = stub;
    }
}

Vector3D<double> Robot::ShiftedEndEffector(size_t qIter)
{
    Transform cumulative = Transform();

    for (size_t i = 0; i < m_dhParams.size(); i++)
    {
        DHParam dhparam = m_dhParams[i];

        if (i == qIter)
            dhparam.SetQ(dhparam.GetQ() + DIFFERENTIAL_DELTA);

        Transform transform = Transform(dhparam.Theta, dhparam.Alpha, dhparam.A, dhparam.D);
        cumulative = Transform::Multiply(cumulative, transform);
    }

    return cumulative.GetDisplacement();
}

void Robot::RecomputeJacobian()
{
    for (size_t qIter = 0; qIter < this->m_dhParams.size(); qIter++)
    {
        Vector3D<double> linearPlusH = this->ShiftedEndEffector(qIter);

        Vector3D<double> linear = this->m_coordinateStubs.back().Position;
        Vector3D<double> linearV = (linearPlusH - linear) * (1.0 / DIFFERENTIAL_DELTA);

        Vector3D<double> angularV = Vector3D<double>(0);
        if (this->m_dhParams[qIter].Type == JointType::REVOLUTE)
            if (qIter == 0)
                angularV = Vector3D<double>(0, 0, 1);
            else
                angularV = this->m_coordinateStubs[qIter].Orientation.GetColumn(2);

        this->m_jacobian.At(0, qIter) = linearV.x;
        this->m_jacobian.At(1, qIter) = linearV.y;
        this->m_jacobian.At(2, qIter) = linearV.z;

        this->m_jacobian.At(3, qIter) = angularV.x;
        this->m_jacobian.At(4, qIter) = angularV.y;
        this->m_jacobian.At(5, qIter) = angularV.z;
    }
}

void Robot::RecomputeStatics()
{
    /* End effector statics */
    Matrix jacobianT = m_jacobian.Transposed().Multiply(-1.0);
    Matrix forceMoment = m_forceMoment.AsColumn();

    Matrix jointResult = Matrix::Multiply(jacobianT, forceMoment);
    this->m_jointForceMoments = jointResult.Flatten();

    /* Base frame equivalent force */
    Transform eeToBase = this->EndEffectorTransform();
    Vector3D<double> posBaseToEE = eeToBase.GetColumn(3);
    Vector3D<double> forceInBase = this->m_forceMoment.Force();

    Vector3D<double> momentAboutEEInBase = eeToBase.TransformPoint(this->m_forceMoment.Moment());
    Vector3D<double> momentDelta = Vector3D<double>::Cross(posBaseToEE, forceInBase);
    Vector3D<double> momentAboutBaseInBase = momentAboutEEInBase - momentDelta;

    this->m_baseForceEquivalent = forceInBase;
    this->m_baseMomentEquivalent = momentAboutBaseInBase;
}

std::vector<double> Robot::GetForwardDKResults()
{
    Matrix jointsColumn = Matrix::CreateCol(this->m_manualJointSpeeds);
    Matrix endEffectorMatrix = Matrix::Multiply(this->m_jacobian, jointsColumn);

    std::vector<double> result = endEffectorMatrix.Flatten();
    return result;
}

std::vector<double> Robot::GetInverseDKResults()
{
    Matrix endEffectorFull = Matrix::CreateCol(this->m_manualEndEffectorVelocity);
    Matrix jacobianAugmented = Matrix::LRJoin(m_jacobian, endEffectorFull);

    jacobianAugmented.GaussJordanEliminate();

    std::vector<double> results;
    results.resize(this->NumJoints(), 0.0);
    m_isFreeVariable.resize(this->NumJoints());

    std::fill(m_isFreeVariable.begin(), m_isFreeVariable.end(), true);

    for (size_t row = 0; row < jacobianAugmented.Rows(); row++)
    {
        size_t freeCol = jacobianAugmented.GetHeadPosition(row);
        m_isFreeVariable[freeCol] = false;
    }

    for (size_t row = 0; row < jacobianAugmented.Rows(); row++)
    {
        results[row] = jacobianAugmented.At(row, jacobianAugmented.Cols() - 1);
        for (size_t col = 0; col < jacobianAugmented.Cols() - 1; col++)
        {
            if (m_isFreeVariable[col])
            {
                double freeDelta = jacobianAugmented.At(row, col) * this->m_manualJointSpeeds[col];
                results[row] -= freeDelta;
            }
        }
    }

    for (size_t col = 0; col < results.size(); col++)
        if (m_isFreeVariable[col])
            results[col] = this->m_manualJointSpeeds[col];

    return results;
}

std::vector<double> Robot::GetDKResults()
{
    std::vector<double> result;

    if (m_dkForwards)
        result = this->GetForwardDKResults();
    else
        result = this->GetInverseDKResults();

    return result;
}

Matrix Robot::GetEEForceMoment() 
{ 
    return this->m_forceMoment.AsColumn();
}

void Robot::Recompute()
{
    this->RecomputeCoordinateStubs();
    this->RecomputeJacobian();
    this->RecomputeStatics();
}

