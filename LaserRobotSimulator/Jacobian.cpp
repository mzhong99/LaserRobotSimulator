#include "Jacobian.hpp"
#include "Robot.hpp"

#include <iomanip>

#define DOUBLE_EPS  (1e-9)

Jacobian::Jacobian()
{
    this->m_data.resize(36);

    for (size_t i = 0; i < 36; i++)
        this->m_data[i] = 0;

    for (size_t i = 0; i < 6; i++)
        this->Set(i, i, 1);
}

std::vector<double> Jacobian::Multiply(const std::vector<double> &values)
{
    std::vector<double> result;
    result.resize(values.size());

    for (size_t row = 0; row < values.size(); row++)
    {
        result[row] = 0;

        for (size_t i = 0; i < values.size(); i++)
            result[row] += this->Get(row, i) * values[i];
    }

    return result;
}

void Jacobian::ForwardMultiply(
    const std::vector<double> &jointSpeeds,
    Vector3D<double> &linearOut,
    Vector3D<double> &angularOut)
{
    std::vector<double> result = this->Multiply(jointSpeeds);

    linearOut = Vector3D<double>(result[0], result[1], result[2]);
    angularOut = Vector3D<double>(result[3], result[4], result[5]);
}

std::vector<double> Jacobian::InverseMultiply(Vector3D<double> linear, Vector3D<double> angular)
{
    std::vector<double> values = { linear.x, linear.y, linear.z, angular.x, angular.y, angular.z };
    return this->Multiply(values);
}

size_t Jacobian::GetHeadPosition(size_t row)
{
    for (size_t col = 0; col < 6; col++)
        if (fabs(this->Get(row, col)) < DOUBLE_EPS)
            this->Set(row, col, 0);
        else
            return col;

    return 6;
}

void Jacobian::SwapRow(Jacobian &lhs, Jacobian &rhs, size_t rowA, size_t rowB)
{
    double temp = 0;

    for (size_t col = 0; col < 6; col++)
    {
        temp = lhs.Get(rowA, col);
        lhs.Set(rowA, col, lhs.Get(rowB, col));
        lhs.Set(rowB, col, temp);

        temp = rhs.Get(rowA, col);
        rhs.Set(rowA, col, rhs.Get(rowB, col));
        rhs.Set(rowB, col, temp);
    }
}

void Jacobian::SortByHead(Jacobian &lhs, Jacobian &rhs)
{
    bool repeat = true;
    while (repeat)
    {
        repeat = false;
        for (size_t i = 1; i < 5; i++)
        {
            size_t prevHeadPos = lhs.GetHeadPosition(i - 1);
            size_t nextHeadPos = lhs.GetHeadPosition(i);

            if (prevHeadPos > nextHeadPos)
            {
                Jacobian::SwapRow(lhs, rhs, i - 1, i);
                repeat = true;
            }
        }
    }
}

void Jacobian::AddRow(Jacobian &lhs, Jacobian &rhs, size_t rowFrom, size_t rowTo, double scaler)
{
    for (size_t col = 0; col < 6; col++)
    {
        lhs.Set(rowTo, col, lhs.Get(rowTo, col) + (lhs.Get(rowFrom, col) * scaler));
        rhs.Set(rowTo, col, rhs.Get(rowTo, col) + (rhs.Get(rowFrom, col) * scaler));
    }
}

void Jacobian::NormalizeToHead(Jacobian &lhs, Jacobian &rhs, size_t row)
{
    size_t baseHeadPos = lhs.GetHeadPosition(row);

    if (baseHeadPos == 6)
        return;

    double baseHead = lhs.Get(row, baseHeadPos);

    for (size_t col = 0; col < 6; col++)
    {
        lhs.Set(row, col, lhs.Get(row, col) / baseHead);
        rhs.Set(row, col, rhs.Get(row, col) / baseHead);
    }
}

void Jacobian::RowReduceDown(Jacobian &lhs, Jacobian &rhs, size_t baseRow)
{
    size_t baseHeadPos = lhs.GetHeadPosition(baseRow);

    if (baseHeadPos == 6)
        return;

    double baseHead = lhs.Get(baseRow, baseHeadPos);
    Jacobian::NormalizeToHead(lhs, rhs, baseRow);

    for (size_t nextRow = baseRow + 1; nextRow < 6; nextRow++)
    {
        double scaler = -1.0 * lhs.Get(nextRow, baseHeadPos) / baseHead;

        if (fabs(lhs.Get(nextRow, baseHeadPos)) > DOUBLE_EPS)
        {
            Jacobian::NormalizeToHead(lhs, rhs, nextRow);
            Jacobian::AddRow(lhs, rhs, baseRow, nextRow, -1.0);
        }
    }
}

void Jacobian::RowReduceUp(Jacobian &lhs, Jacobian &rhs, size_t baseRow)
{
    size_t baseHeadPos = lhs.GetHeadPosition(baseRow);

    if (baseHeadPos == 6)
        return;

    double baseHead = lhs.Get(baseRow, baseHeadPos);

    for (size_t prevRow = 0; prevRow < baseRow; prevRow++)
    {
        double scaler = -1.0 * lhs.Get(prevRow, baseHeadPos) / baseHead;
        Jacobian::AddRow(lhs, rhs, baseRow, prevRow, scaler);
    }
}

Jacobian Jacobian::Inverted()
{
    std::vector<double> dataCopy = this->m_data;

    Jacobian copy = *this;
    Jacobian inverse = Jacobian();

    for (size_t row = 0; row < 6; row++)
    {
        Jacobian::SortByHead(copy, inverse);
        Jacobian::RowReduceDown(copy, inverse, row);
    }

    if (DebugCheck())
        std::cout << "row-echelon form:" << std::endl;
    DebugPrint(copy, inverse);

    for (size_t rowIter = 0; rowIter < 6; rowIter++)
    {
        size_t row = 5 - rowIter;
        Jacobian::RowReduceUp(copy, inverse, row);
    }

    return inverse;
}

bool Jacobian::DebugCheck()
{
    return Simulator::Input().KeyTapped(SDLK_3) && Simulator::Input().KeyPressed(SDLK_LCTRL);
}

void Jacobian::DebugPrint(Jacobian &lhs, Jacobian &rhs)
{
    if (Simulator::Input().KeyTapped(SDLK_3) && Simulator::Input().KeyPressed(SDLK_LCTRL))
    {
        for (size_t row = 0; row < 6; row++)
        {
            std::cout << "[ ";
            for (size_t col = 0; col < 6; col++)
                std::cout << std::fixed << std::setprecision(3) << lhs.Get(row, col) << " ";
            std::cout << "] ";

            std::cout << "[ ";
            for (size_t col = 0; col < 6; col++)
                std::cout << rhs.Get(row, col) << " ";
            std::cout << "]" << std::endl;
        }
        std::cout << std::endl;
    }
}

Jacobian Jacobian::Multiply(Jacobian &lhs, Jacobian &rhs)
{
    Jacobian result = Jacobian();

    for (int row = 0; row < 6; row++)
    {
        for (int col = 0; col < 6; col++)
        {
            double sum = 0;

            for (int i = 0; i < 6; i++)
                sum += lhs.Get(row, i) * rhs.Get(i, col);

            result.Set(row, col, sum);
        }
    }

    return result;
}

void Jacobian::Set(size_t row, size_t col, double val)
{
    this->m_data[(row * 6) + col] = val;
}

double Jacobian::Get(size_t row, size_t col)
{
    return this->m_data[(row * 6) + col];
}
