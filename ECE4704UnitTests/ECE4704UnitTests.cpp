#include "pch.h"
#include "CppUnitTest.h"
#include "../LaserRobotSimulator/Matrix.hpp"

#include "../LaserRobotSimulator/JointPath.hpp"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace ECE4704UnitTests
{
    TEST_CLASS(ECE4704UnitTests)
    {
    public:
        TEST_METHOD(JointPathing)
        {
            JointPath path = JointPath(0, 3, 2);
            std::vector<double> expected = { 3.75, -2.8125, 0.5625 };
            std::vector<double> result = path.GetConstants();

            for (size_t i = 0; i < expected.size(); i++)
                Assert::IsTrue(fabs(expected[i] - result[i]) < 1e-9);
        }
    };
}
