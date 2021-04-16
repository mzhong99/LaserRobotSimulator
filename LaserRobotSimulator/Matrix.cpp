#include "Matrix.hpp"
#include "Simulator.hpp"

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
        for (size_t col = 0; col < rhs.Cols(); col++)
        {
            result.At(row, col) = lhs.At(row, col);
            result.At(row, col + lhs.Cols()) = rhs.At(row, col);
        }
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

    return augmented.SubMatrix(this->Rows(), augmented.Rows(), this->Cols(), augmented.Cols());
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
            submatrix.At(row, col) = this->At(rowLow + row, colLow + col);

    return submatrix;
}

size_t Matrix::GetHeadPosition(size_t row)
{
    for (size_t col = 0; col < this->Cols(); col++)
        if (fabs(this->At(row, col)) < DOUBLE_EPS)
            this->At(row, col) = 0;
        else
            return col;

    return this->Cols();
}

void Matrix::SwapRow(size_t rowA, size_t rowB)
{
    for (size_t col = 0; col < this->Cols(); col++)
        std::swap(this->At(rowA, col), this->At(rowB, col));
}

void Matrix::SortByHead()
{
    if (this->Rows() < 2)
        return;

    for (size_t row = 0; row < this->Rows() - 1; row++)
    {
        for (size_t i = 0; i < this->Rows() - i - 1; i++)
        {
            size_t currHeadPos = this->GetHeadPosition(i);
            size_t nextHeadPos = this->GetHeadPosition(i + 1);

            if (currHeadPos > nextHeadPos)
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

void Matrix::RowReduceDown(size_t baseRow)
{
    size_t baseHeadPos = this->GetHeadPosition(baseRow);

    if (baseHeadPos == this->Cols())
        return;

    this->NormalizeToHead(baseRow);

    double baseHead = this->At(baseRow, baseHeadPos);
    for (size_t nextRow = baseRow + 1; nextRow < this->Rows(); nextRow++)
    {
        this->NormalizeToHead(nextRow);
        if (DebugCheck())
        {
            this->DebugPrint();
            std::cout << std::endl;
        }

        if (fabs(this->At(nextRow, baseHeadPos) > DOUBLE_EPS))
            this->SubtractRow(baseRow, nextRow);
    }
}

void Matrix::RowReduceUp(size_t baseRow)
{
    for (size_t prevRow = 0; prevRow < baseRow; prevRow++)
    {
        size_t baseHeadPos = this->GetHeadPosition(baseRow);
        this->NormalizeRowToElement(prevRow, baseRow, baseHeadPos);
        this->SubtractRow(baseRow, prevRow);
        this->NormalizeToHead(prevRow);
    }
}

void Matrix::GaussJordanEliminate()
{
    if (DebugCheck())
    {
        std::cout << "Begin Gauss Jordan..." << std::endl;
        std::cout << "----------------------------------------" << std::endl;
        this->DebugPrint();
        std::cout << std::endl;
    }

    for (size_t row = 0; row < this->Rows(); row++)
    {
        if (DebugCheck())
            std::cout << "Row Reduce Down [" << row << "]" << std::endl;

        this->SortByHead();
        if (DebugCheck())
        {
            std::cout << "Sorted By Head" << std::endl;

            this->DebugPrint();
            std::cout << std::endl;
        }

        this->RowReduceDown(row);

        if (DebugCheck())
        {
            this->DebugPrint();
            std::cout << std::endl;
        }
    }

    if (DebugCheck())
    {
        std::cout << "Row-Echelon Form" << std::endl;
        std::cout << "----------------------------------------" << std::endl;

        this->DebugPrint();
        std::cout << std::endl;
    }

    for (size_t row = 0; row < this->Rows(); row++)
    {
        if (DebugCheck())
            std::cout << "Row Reduce Up [" << row << "]" << std::endl;

        this->RowReduceUp(this->Rows() - row - 1);

        if (DebugCheck())
        {
            this->DebugPrint();
            std::cout << std::endl;
        }
    }
}

bool Matrix::DebugCheck()
{
    return Simulator::Input().KeyTapped(SDLK_m);
}

void Matrix::DebugPrint()
{
    char buf[256];
    const char *sep = "";

    for (size_t row = 0; row < this->Rows(); row++)
    {
        std::cout << "[";
        sep = "";

        for (size_t col = 0; col < this->Cols(); col++)
        {
            snprintf(buf, 255, "%s%+4.6lf", sep, this->At(row, col));
            buf[255] = '\0';

            std::cout << buf;
            sep = ", ";
        }
        std::cout << "]" << std::endl;
    }
    std::cout << std::endl;
}
