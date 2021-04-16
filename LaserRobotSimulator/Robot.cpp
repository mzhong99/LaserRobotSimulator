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

Vector3D<double> CoordinateStub::AngularPosition()
{
    Vector3D<double> angular;

    angular.x = atan2(this->Position.z, this->Position.y);
    angular.y = atan2(this->Position.x, this->Position.z);
    angular.z = atan2(this->Position.y, this->Position.x);

    return angular;
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
    m_dhParams.push_back(DHParam::CreatePrismatic(M_PI / 2.0, M_PI / 2.0, 0));
    m_dhParams.push_back(DHParam::CreatePrismatic(M_PI / 2.0, M_PI / 2.0, 0));
    m_dhParams.push_back(DHParam::CreateRevolute(M_PI / 2.0, 0, 0));
    m_dhParams.push_back(DHParam::CreateRevolute(-M_PI / 2.0, 0, 0));
    m_dhParams.push_back(DHParam::CreatePrismatic(0, 0, 0));

    m_coordinateStubs.resize(m_dhParams.size());
    m_manualJointSpeeds.resize(m_dhParams.size());
    m_manualEndEffectorVelocity.resize(3);

    m_jacobian = Matrix(6, m_dhParams.size());
    m_jointForceMoments.resize(m_dhParams.size());
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

void Robot::ShiftedEndEffector(
    size_t qIter,
    Vector3D<double> &linearOut,
    Vector3D<double> &angularOut)
{
    Transform cumulative = Transform();

    for (size_t i = 0; i < m_dhParams.size(); i++)
    {
        DHParam dhparam = m_dhParams[i];

        if (i == qIter)
        {
            switch (dhparam.Type)
            {
                case JointType::REVOLUTE  : dhparam.Theta += DIFFERENTIAL_DELTA; break; 
                case JointType::PRISMATIC : dhparam.D     += DIFFERENTIAL_DELTA; break;
                default: break;
            }
        }

        Transform transform = Transform(dhparam.Theta, dhparam.Alpha, dhparam.A, dhparam.D);
        if (i == qIter && qIter == 5)
        {
            if (Simulator::Input().KeyTapped(SDLK_2) && Simulator::Input().KeyPressed(SDLK_LCTRL))
            {
                std::cout << std::fixed << std::setprecision(8);
                std::cout << "Last Transform:" << std::endl;
                for (int row = 0; row < 4; row++)
                {
                    std::cout << "[ ";
                    for (int col = 0; col < 4; col++)
                        std::cout << transform.At(row, col) << " ";
                    std::cout << "]" << std::endl;
                }

                Transform transform = Transform(dhparam.Theta, dhparam.Alpha, dhparam.A, dhparam.D);
            }
        }

        cumulative = Transform::Multiply(cumulative, transform);
    }

    linearOut = Vector3D<double>(cumulative.GetDisplacement());
    angularOut = Vector3D<double>();

    angularOut.x = atan2(linearOut.z, linearOut.y);
    angularOut.y = atan2(linearOut.x, linearOut.z);
    angularOut.z = atan2(linearOut.y, linearOut.x);
}

void Robot::RecomputeJacobian()
{
    for (size_t qIter = 0; qIter < this->m_dhParams.size(); qIter++)
    {
        Vector3D<double> linearPlusH, angularPlusH;
        this->ShiftedEndEffector(qIter, linearPlusH, angularPlusH);

        Vector3D<double> linear = this->m_coordinateStubs.back().Position;
        Vector3D<double> linearDelta = (linearPlusH - linear) * (1.0 / DIFFERENTIAL_DELTA);

        Vector3D<double> angular = this->m_coordinateStubs.back().AngularPosition();
        Vector3D<double> angularDelta = (angularPlusH - angular) * (1.0 / DIFFERENTIAL_DELTA);

        this->m_jacobian.At(0, qIter) = linearDelta.x;
        this->m_jacobian.At(1, qIter) = linearDelta.y;
        this->m_jacobian.At(2, qIter) = linearDelta.z;

        this->m_jacobian.At(3, qIter) = angularDelta.x;
        this->m_jacobian.At(4, qIter) = angularDelta.y;
        this->m_jacobian.At(5, qIter) = angularDelta.z;
    }
}

void Robot::RecomputeStatics()
{
    Matrix jacobianT = m_jacobian.Transposed().Multiply(-1.0);
    Matrix forceMoment = m_forceMoment.AsColumn();

    Matrix jointResult = Matrix::Multiply(jacobianT, forceMoment);
    this->m_jointForceMoments = jointResult.Flatten();
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
    // return this->m_jacobian.LeastSquaresMultiply(this->m_manualEndEffectorVelocity);
    // Jacobian inverted = this->m_jacobian.Inverted();

    // Vector3D<double> linear = Vector3D<double>(
    //     this->m_manualEndEffectorVelocity[0],
    //     this->m_manualEndEffectorVelocity[1],
    //     this->m_manualEndEffectorVelocity[2]);

    // Vector3D<double> angular = Vector3D<double>(
    //     this->m_manualEndEffectorVelocity[3],
    //     this->m_manualEndEffectorVelocity[4],
    //     this->m_manualEndEffectorVelocity[5]);

    // std::vector<double> result = inverted.InverseMultiply(linear, angular);
    // return result;
    Matrix bcol = Matrix::CreateCol(this->m_manualEndEffectorVelocity);

    if (Matrix::DebugCheck())
        this->m_jacobian.DebugPrint();

    Matrix AT = this->m_jacobian.Transposed();
    if (Matrix::DebugCheck())
        AT.DebugPrint();

    Matrix ATA = Matrix::Multiply(AT, this->m_jacobian);
    if (Matrix::DebugCheck())
    {
        ATA.DebugPrint();
        std::cout << "DEFAULT" << std::endl;
    }

    if (Matrix::DebugCheck())
        Matrix::Multiply(ATA, bcol).DebugPrint();

    Matrix ATAinverted = ATA.Inverted();
    Matrix left = Matrix::Multiply(ATAinverted, AT);

    return Matrix::Multiply(left, bcol).Flatten();
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

void Robot::Recompute()
{
    this->RecomputeCoordinateStubs();
    this->RecomputeJacobian();
    this->RecomputeStatics();
}

