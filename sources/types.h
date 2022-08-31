
#pragma once

#include <opencv2/core.hpp>

template <typename T>
inline bool isClose(const T &x, const T &y)
{
    return std::abs(x - y) < std::abs(1e-4 * std::min(x, y));
}

struct DistortionCoefficients
{
    double k1;
    double k2;
    double p1;
    double p2;
    double k3;
    double cx;
    double cy;

    friend std::ostream &operator<<(std::ostream &out, const DistortionCoefficients &kd)
    {
        out << kd.k1 << ", " << kd.k2 << ", " << kd.p1 << ", " << kd.p2 << ", " << kd.k3 << ", " << kd.cx << ", " << kd.cy << std::endl;
        return out;
    }

    bool operator==(const DistortionCoefficients &other) const
    {
        return isClose(k1, other.k1) && isClose(k2, other.k2) && isClose(p1, other.p1) && isClose(k3, other.k3) && isClose(cx, other.cx) && isClose(cy, other.cy);
    }
};

struct DistortionMapping
{
    double off_x;
    double off_y;
    double ext_x;
    double ext_y;
    cv::Mat map_x;
    cv::Mat map_y;
};
