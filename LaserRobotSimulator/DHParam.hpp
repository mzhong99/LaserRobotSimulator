#pragma once

enum class JointType { PRISMATIC, REVOLUTE, NONE };

struct DHParam
{
    double Theta;
    double Alpha;
    double A;
    double D;

    JointType Type;

    DHParam(): 
        Theta(0), Alpha(0), A(0), D(0), Type(JointType::NONE) {}

    DHParam(double theta, double alpha, double a, double d):
        Theta(theta), Alpha(alpha), A(a), D(d), Type(JointType::NONE) {}

    static DHParam CreateRevolute(double alpha, double a, double d)
    {
        DHParam param;

        param.Alpha = alpha;
        param.A = a;
        param.D = d;
        param.Theta = 1.0;

        param.Type = JointType::REVOLUTE;

        return param;
    }

    static DHParam CreatePrismatic(double theta, double alpha, double a)
    {
        DHParam param;

        param.Theta = theta;
        param.Alpha = alpha;
        param.A = a;
        param.D = 3.0;

        param.Type = JointType::PRISMATIC;

        return param;
    }

    double GetQ()
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

    void SetQ(double val)
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
};
