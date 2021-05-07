#include "Matrix.hpp"

#include <cstdio>

#define DOUBLE_EPS (1e-14)

Matrix::Matrix(size_t rows, size_t cols): m_rows(rows), m_cols(cols) 
{
    m_data.resize(m_rows * m_cols, 0); 
}

Matrix Matrix::CreateCol(std::vector<double> values)
{
    Matrix matrix = Matrix(values.size(), 1);
    matrix.m_data = values;
    return matrix;
}

Matrix Matrix::CreateRow(std::vector<double> values)
{
    Matrix matrix = Matrix(1, values.size());
    matrix.m_data = values;
    return matrix;
}

Matrix Matrix::Multiply(Matrix &lhs, Matrix &rhs)
{
    if (lhs.Cols() != rhs.Rows())
        return Matrix();

    Matrix result = Matrix(lhs.Rows(), rhs.Cols());

    for (size_t row = 0; row < result.Rows(); row++)
        for (size_t col = 0; col < result.Cols(); col++)
            for (size_t i = 0; i < lhs.Cols(); i++)
                result.At(row, col) += lhs.At(row, i) * rhs.At(i, col);

    return result;
}

Matrix Matrix::Multiply(double scalar)
{
    Matrix copy = *this;

    for (size_t row = 0; row < this->Rows(); row++)
        for (size_t col = 0; col < this->Cols(); col++)
            copy.At(row, col) = scalar * this->At(row, col);

    return copy;
}

Matrix Matrix::LRJoin(Matrix &lhs, Matrix &rhs)
{
    if (lhs.Rows() != rhs.Rows())
        return Matrix();

    Matrix result = Matrix(lhs.Rows(), lhs.Cols() + rhs.Cols());

    for (size_t row = 0; row < lhs.Rows(); row++)
    {
        for (size_t col = 0; col < lhs.Cols(); col++)
            result.At(row, col) = lhs.At(row, col);

        for (size_t col = 0; col < rhs.Cols(); col++)
            result.At(row, col + lhs.Cols()) = rhs.At(row, col);
    }

    return result;
}

Matrix Matrix::Identity(size_t n)
{
    Matrix result = Matrix(n, n);

    for (size_t i = 0; i < n; i++)
        result.At(i, i) = 1.0;

    return result;
}

Matrix Matrix::Inverted()
{
    if (this->Rows() != this->Cols())
        return Matrix();

    Matrix identity = Matrix::Identity(this->Rows());

    Matrix augmented = Matrix::LRJoin(*this, identity);
    augmented.GaussJordanEliminate();

    return augmented.SubMatrix(0, this->Rows(), this->Cols(), augmented.Cols());
}

Matrix Matrix::Transposed()
{
    Matrix transpose = Matrix(this->Cols(), this->Rows());

    for (size_t row = 0; row < this->Rows(); row++)
        for (size_t col = 0; col < this->Cols(); col++)
            transpose.At(col, row) = this->At(row, col);

    return transpose;
}

Matrix Matrix::SubMatrix(size_t rowLow, size_t rowHigh, size_t colLow, size_t colHigh)
{
    if (rowLow >= rowHigh || colLow >= colHigh)
        return Matrix();

    Matrix submatrix = Matrix(rowHigh - rowLow, colHigh - colLow);

    for (size_t row = rowLow; row < rowHigh; row++)
        for (size_t col = colLow; col < colHigh; col++)
            submatrix.At(row - rowLow, col - colLow) = this->At(row, col);

    return submatrix;
}

size_t Matrix::GetHeadPosition(size_t row)
{
    for (size_t col = 0; col < this->Cols(); col++)
        if (fabs(this->At(row, col)) < DOUBLE_EPS)
            this->At(row, col) = 0.0;
        else
            return col;

    return this->Cols();
}

void Matrix::SwapRow(size_t rowA, size_t rowB)
{
    for (size_t col = 0; col < this->Cols(); col++)
        std::swap(this->At(rowA, col), this->At(rowB, col));
}

void Matrix::MoveZerosDown(size_t col)
{
    if (this->Rows() < 2)
        return;

    int left = (int)col;
    int right = this->Rows() - 1;

    while (left < (int)this->Rows() && fabs(this->At(left, col)) > DOUBLE_EPS)
        left++;

    while (right > -1 && fabs(this->At(right, col)) < DOUBLE_EPS)
        right--;

    while (left < right)
    {
        this->SwapRow((size_t)left, (size_t)right);

        while (left < (int)this->Rows() && fabs(this->At(left, col)) > DOUBLE_EPS)
            left++;

        while (right > -1 && fabs(this->At(right, col)) < DOUBLE_EPS)
            right--;
    }
}

void Matrix::SortByHead()
{
    if (this->Rows() < 2)
        return;

    for (size_t row = 0; row < this->Rows(); row++)
    {
        for (size_t i = 0; i < this->Rows() - 1; i++)
        {
            size_t currHeadPos = this->GetHeadPosition(i);
            size_t nextHeadPos = this->GetHeadPosition(i + 1);

            if (currHeadPos > nextHeadPos)
                this->SwapRow(i, i + 1);
            else if (currHeadPos < this->Cols() && nextHeadPos < this->Cols())
                if (this->At(i, currHeadPos) > this->At(i + 1, nextHeadPos))
                    this->SwapRow(i, i + 1);
        }
    }
}

void Matrix::AddRow(size_t rowFrom, size_t rowTo)
{
    for (size_t col = 0; col < this->Cols(); col++)
        this->At(rowTo, col) += this->At(rowFrom, col);
}

void Matrix::SubtractRow(size_t rowFrom, size_t rowTo)
{
    for (size_t col = 0; col < this->Cols(); col++)
        this->At(rowTo, col) -= this->At(rowFrom, col);
}

void Matrix::NormalizeToHead(size_t row)
{
    size_t headPos = this->GetHeadPosition(row);

    if (headPos >= this->Cols())
        return;

    this->NormalizeRowToElement(row, row, headPos);
}

void Matrix::NormalizeRowToElement(size_t baseRow, size_t row, size_t col)
{
    double value = this->At(row, col);

    for (size_t i = 0; i < this->Cols(); i++)
        this->At(baseRow, i) /= value;
}

Matrix Matrix::GetRow(size_t row)
{
    if (row >= this->Rows())
        return Matrix();

    Matrix copy = Matrix(1, this->Cols());
    for (size_t col = 0; col < this->Cols(); col++)
        copy.At(0, col) = this->At(row, col);

    return copy;
}

Matrix Matrix::GetCol(size_t col)
{
    if (col >= this->Cols())
        return Matrix();

    Matrix copy = Matrix(this->Rows(), 1);
    for (size_t row = 0; row < this->Rows(); row++)
        copy.At(row, 0) = this->At(row, col);

    return copy;
}

Matrix Matrix::FromVector3D(Vector3D<double> vec3D, size_t length, double fillValue)
{
    Matrix matrix = Matrix(length, 1);

    if (length > 0)
        matrix.At(0, 0) = vec3D.x;

    if (length > 1)
        matrix.At(1, 0) = vec3D.y;

    if (length > 2)
        matrix.At(2, 0) = vec3D.z;

    for (size_t i = 3; i < length; i++)
        matrix.m_data[i] = fillValue;

    return matrix;
}

Vector3D<double> Matrix::ToVector3D()
{
    if (this->Rows() > 1 && this->Cols() > 1)
        return Vector3D<double>();

    Vector3D<double> answer = Vector3D<double>(0);

    if (this->m_data.size() > 0)
        answer.x = this->m_data[0];

    if (this->m_data.size() > 1)
        answer.y = this->m_data[1];

    if (this->m_data.size() > 1)
        answer.z = this->m_data[2];

    return answer;
}

void Matrix::RowReduceDown(size_t baseRow)
{
    size_t baseHeadPos = this->GetHeadPosition(baseRow);

    if (baseHeadPos == this->Cols())
        return;

    this->NormalizeToHead(baseRow);

    double baseHead = this->At(baseRow, baseHeadPos);
    for (size_t nextRow = baseRow + 1; nextRow < this->Rows(); nextRow++)
    {
        if (fabs(this->At(nextRow, baseRow)) < DOUBLE_EPS)
            continue;

        this->NormalizeToHead(baseRow);

        double nextVal = this->At(nextRow, baseRow);
        for (size_t col = 0; col < this->Cols(); col++)
            this->At(baseRow, col) *= nextVal;

        this->SubtractRow(baseRow, nextRow);
        this->NormalizeToHead(baseRow);
    }
}

void Matrix::RowReduceUp(size_t baseRow)
{
    for (size_t prevRow = 0; prevRow < baseRow; prevRow++)
    {
        if (fabs(this->At(prevRow, baseRow)) < DOUBLE_EPS)
            continue;

        this->NormalizeToHead(baseRow);

        double prevVal = this->At(prevRow, baseRow);
        for (size_t col = 0; col < this->Cols(); col++)
            this->At(baseRow, col) *= prevVal;

        this->SubtractRow(baseRow, prevRow);
        this->NormalizeToHead(baseRow);
    }
}

void Matrix::GaussJordanEliminate()
{
    for (size_t row = 0; row < this->Rows(); row++)
    {
        this->MoveZerosDown(row);

        for (size_t rowAfter = row; rowAfter < this->Rows(); rowAfter++)
            this->NormalizeToHead(rowAfter);

        this->RowReduceDown(row);
    }

    for (size_t row = 0; row < this->Rows(); row++)
        this->RowReduceUp(this->Rows() - row - 1);
}

std::vector<double> Matrix::RREFWithFreeVariables(
    const std::vector<double> &freeVars, std::vector<bool> &isFreeVarOut)
{
    if (this->Cols() != freeVars.size() + 1)
        return std::vector<double>();

    isFreeVarOut = std::vector<bool>(freeVars.size(), true);
    std::vector<double> results = std::vector<double>(freeVars.size(), 0.0);

    this->GaussJordanEliminate();

    for (size_t row = 0; row < this->Rows(); row++)
    {
        size_t freeCol = this->GetHeadPosition(row);

        /* Matrix is singular when there's a pivot in the answer column */
        if (freeCol + 1 == this->Cols())
            return std::vector<double>();

        if (freeCol < isFreeVarOut.size())
            isFreeVarOut[freeCol] = false;
    }

    for (size_t row = 0; row < this->Rows(); row++)
    {
        results[row] = this->At(row, this->Cols() - 1);
        for (size_t col = 0; col < this->Cols() - 1; col++)
        {
            if (isFreeVarOut[col])
                results[row] -= this->At(row, col) * freeVars[col];
        }
    }

    for (size_t col = 0; col < results.size(); col++)
        if (isFreeVarOut[col])
            results[col] = freeVars[col];

    return results;
}

std::ostream &operator<<(std::ostream &os, Matrix &matrix)
{
    for (size_t i = 0; i < matrix.Rows(); i++)
    {
        os << "[";
        const char *sep = "";

        for (size_t j = 0; j < matrix.Cols(); j++)
        {
            char buf[256];

            snprintf(buf, 256, "%s%+.8f", sep, matrix.At(i, j));
            os << buf;

            sep = ", ";
        }

        os << "]" << std::endl;
    }

    return os;
}
