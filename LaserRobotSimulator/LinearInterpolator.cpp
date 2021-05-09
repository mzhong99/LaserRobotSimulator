#include "LinearInterpolator.hpp"

double LinearInterpolator::Interpolate(double time, bool approximate)
{
    if (time <= 0)
        return m_values[0];

    if (time >= m_times.back())
        return m_values.back();

    size_t left = 0;
    size_t right = m_times.size() - 1;
    size_t mid = left + (right - left) / 2;

    while (left < right)
    {
        mid = left + (right - left) / 2;
        double found = m_times[mid];

        if (found == time)
            return found;

        if (found > time)
            right = mid - 1;

        if (found < time)
            left = mid + 1;
    }

    double midVal = m_values[mid];
    if (!approximate)
        return midVal;

    if (time < midVal)
    {
        left = mid - 1;
        right = mid;
    }
    else
    {
        left = mid;
        right = mid + 1;
    }

    double leftVal = m_values[left];
    double rightVal = m_values[right];

    double leftTime = m_times[left];
    double rightTime = m_times[right];

    return (((rightVal - leftVal) / (rightTime - leftTime)) * (time - leftTime)) + leftVal;
}
