#pragma once

#include <vector>
#include <algorithm>
#include <iostream>

#include <cmath>
#include <cstdio>

class Matrix
{
private:
    std::vector<double> m_data;
    size_t m_rows;
    size_t m_cols;

    void SortByHead();
    void SwapRow(size_t rowA, size_t rowB);

    void NormalizeToHead(size_t row);
    void NormalizeRowToElement(size_t baseRow, size_t row, size_t col);
    void RowReduceDown(size_t baseRow);
    void RowReduceUp(size_t baseRow);

    void AddRow(size_t rowFrom, size_t rowTo);
    void SubtractRow(size_t rowFrom, size_t rowTo);

public:
    Matrix(): Matrix(0, 0) {}
    Matrix(size_t rows, size_t cols);

    static Matrix CreateCol(std::vector<double> values);
    static Matrix CreateRow(std::vector<double> values);

    size_t Rows() { return m_rows; }
    size_t Cols() { return m_cols; }

    double &At(size_t row, size_t col)
    { return this->m_data[(this->Cols() * row) + col]; }

    static Matrix Identity(size_t n);
    static Matrix Multiply(Matrix &lhs, Matrix &rhs);
    static Matrix LRJoin(Matrix &lhs, Matrix &rhs);

    Matrix Inverted();
    Matrix Transposed();

    Matrix Multiply(double scalar);

    void GaussJordanEliminate();
    size_t GetHeadPosition(size_t row);
    
    /* Excludes the high element, includes the low one */
    Matrix SubMatrix(size_t rowLow, size_t rowHigh, size_t colLow, size_t colHigh);
    std::vector<double> Flatten() { return m_data; }

    static bool DebugCheck();
    void DebugPrint();

    friend std::ostream &operator<<(std::ostream &os, Matrix &matrix);
};
