#pragma once

#include <vector>

class LinearInterpolator
{
private:
    std::vector<double> m_times;
    std::vector<double> m_values;

public:
    LinearInterpolator() = default;
    LinearInterpolator(std::vector<double> times, std::vector<double> values):
        m_times(times), m_values(values) {}

    void AddTime(double time, double value) { m_times.push_back(time); m_values.push_back(value); }
    double Interpolate(double time, bool approximate = true);
};

