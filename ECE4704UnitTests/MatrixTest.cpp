#include "pch.h"
#include "CppUnitTest.h"
#include "../LaserRobotSimulator/Matrix.hpp"
#include <vector>
#include <cmath>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace ECE4704UnitTests
{
    TEST_CLASS(MatrixTests)
    {
    public:
        TEST_METHOD(RowReduction1)
        {
            std::vector<double> data =
            {
                3, 2, -1,  1,
                1, 2,  2,  0,
                2, 1, -3, -1
            };

            std::stringstream ss;
            ss << "DANK MEME" << std::endl;

            Matrix mat3x4 = Matrix(3, 4, data);
            ss << mat3x4 << std::endl;

            size_t rows = mat3x4.Rows();
            size_t cols = mat3x4.Cols();

            mat3x4.GaussJordanEliminate();
            ss << mat3x4 << std::endl;
            Logger::WriteMessage(ss.str().c_str());

            std::vector<double> expected =
            {
                1, 0, 0,  2.0,
                0, 1, 0, -2.0,
                0, 0, 1,  1.0
            };
            std::vector<double> result = mat3x4.Flatten();

            for (size_t i = 0; i < data.size(); i++)
                Assert::IsTrue(fabs(expected[i] - result[i]) < 1e-3);
        }

        TEST_METHOD(RowReduction2)
        {
            std::vector<double> data =
            {
                0.692710, 0.095961, 0.021558, 0.412534,
                0.996655, 0.139207, 0.591997, 0.111698,
                0.764028, 0.970097, 0.994772, 0.087246
            };

            std::stringstream ss;
            ss << "DANK MEME" << std::endl;

            Matrix mat3x4 = Matrix(3, 4, data);
            ss << mat3x4 << std::endl;

            size_t rows = mat3x4.Rows();
            size_t cols = mat3x4.Cols();

            mat3x4.GaussJordanEliminate();
            ss << mat3x4 << std::endl;
            Logger::WriteMessage(ss.str().c_str());

            std::vector<double> expected =
            {
                1, 0, 0,  0.54740,
                0, 1, 0,  0.54073,
                0, 0, 1, -0.86004
            };
            std::vector<double> result = mat3x4.Flatten();

            for (size_t i = 0; i < data.size(); i++)
                Assert::IsTrue(fabs(expected[i] - result[i]) < 1e-3);
        }

        TEST_METHOD(RowReduction3)
        {
            std::vector<double> data =
            {
                0.000000, 0.095961, 0.021558, 0.412534,
                0.996655, 0.139207, 0.591997, 0.111698,
                0.764028, 0.970097, 0.994772, 0.087246
            };

            std::stringstream ss;
            ss << "DANK MEME" << std::endl;

            Matrix mat3x4 = Matrix(3, 4, data);
            ss << mat3x4 << std::endl;

            size_t rows = mat3x4.Rows();
            size_t cols = mat3x4.Cols();

            mat3x4.GaussJordanEliminate();
            ss << mat3x4 << std::endl;
            Logger::WriteMessage(ss.str().c_str());

            std::vector<double> expected =
            {
                1, 0, 0,   5.52710,
                0, 1, 0,   6.70106,
                0, 0, 1, -10.69219,
            };
            std::vector<double> result = mat3x4.Flatten();

            for (size_t i = 0; i < data.size(); i++)
                Assert::IsTrue(fabs(expected[i] - result[i]) < 1e-3);
        }

        TEST_METHOD(RowReduction4)
        {
            std::vector<double> data =
            {
                +0.000, +1.000, +0.000, +0.000, -0.00000000, +0.00000000, +0.0,
                +0.000, -0.000, +1.000, +0.000, +0.00000000, -0.00000000, +0.0,
                +1.000, +0.000, -0.000, -0.000, +0.00000000, +0.00000000, +1.5,
                +0.000, +0.000, +0.000, -0.000, +0.84147098, +0.45464871, +0.0,
                +0.000, +0.000, +0.000, +0.000, -0.54030231, +0.70807342, +0.0,
                +0.000, +0.000, +0.000, +1.000, +0.00000000, -0.54030231, +1.5
            };

            std::stringstream ss;
            ss << "DANK MEME" << std::endl;

            Matrix matrix = Matrix(6, 7, data);
            ss << matrix << std::endl;

            size_t rows = matrix.Rows();
            size_t cols = matrix.Cols();

            matrix.GaussJordanEliminate();
            ss << matrix << std::endl;
            Logger::WriteMessage(ss.str().c_str());

            std::vector<double> expected =
            {
                1, 0, 0, 0, 0, 0, 1.5,
                0, 1, 0, 0, 0, 0, 0,
                0, 0, 1, 0, 0, 0, 0,
                0, 0, 0, 1, 0, 0, 1.5,
                0, 0, 0, 0, 1, 0, 0,
                0, 0, 0, 0, 0, 1, 0
            };
            std::vector<double> result = matrix.Flatten();

            for (size_t i = 0; i < data.size(); i++)
                Assert::IsTrue(fabs(expected[i] - result[i]) < 1e-3);
        }
    };
}

