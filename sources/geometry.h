

#pragma once

#include <utility>
#include <type_traits>
#include <numeric>
#include <iostream>
#include <cmath>

#include <opencv2/core.hpp>

template <typename T>
inline bool isZero(const T &x)
{
    return std::abs(x) < std::numeric_limits<double>::epsilon();
}

namespace geometry
{
    namespace detail
    {
        template <typename T, template <typename> class LineType>
        static constexpr bool intersection(const LineType<T> &l0, const LineType<T> &l1, cv::Point_<T> &point)
        {
            double x1 = l0.p0.x;
            double y1 = l0.p0.y;
            double x2 = l0.p1.x;
            double y2 = l0.p1.y;
            double x3 = l1.p0.x;
            double y3 = l1.p0.y;
            double x4 = l1.p1.x;
            double y4 = l1.p1.y;

            double c1 = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
            if (isZero(c1))
                return false;

            double c2 = x1 * y2 - y1 * x2;
            double c3 = x3 * y4 - y3 * x4;
            point.x = (c2 * (x3 - x4) - (x1 - x2) * c3) / c1;
            point.y = (c2 * (y3 - y4) - (y1 - y2) * c3) / c1;
            return true;
        }

        template <typename T, template <typename> class LineType>
        static constexpr bool parallel(const LineType<T> l0, const LineType<T> &l1)
        {
            return isZero(l0.slope - l1.slope);
        }

        template <typename T, template <typename> class LineType>
        static constexpr double getSegmentDistance(const LineType<T> &line, const cv::Point_<T> &point)
        {
            const double l2 = pow((line.p0.x - line.p1.x), 2) + pow((line.p0.y - line.p1.y), 2);
            if (isZero(l2))
                return sqrt(pow((line.p0.x - point.x), 2) + pow((line.p0.y - point.y), 2));

            const auto d0 = point - line.p0;
            const auto d1 = line.p1 - line.p0;
            const auto dot = d0.x * d1.x + d0.y * d1.y;
            const auto t = std::max(0., std::min(1., dot / l2));
            const auto proj = line.p0 + t * d0;

            return sqrt(pow((proj.x - point.x), 2) + pow((proj.y - point.y), 2));
        }

        template <typename T, template <typename> class LineType>
        static constexpr double getLineDistance(const LineType<T> &line, const cv::Point_<T> &point)
        {
            double a = line.p1.y - line.p0.y;
            double b = line.p0.x - line.p1.x;
            double c = line.p1.x * line.p0.y - line.p0.x * line.p1.y;
            double d = sqrt(pow(a, 2) + pow(b, 2));

            if (isZero(d))
                return 0.0;
            return abs(a * point.x + b * point.y + c) / d;
        }
    }

    template <typename T>
    struct Line_
    {
        Line_() = delete;

        explicit Line_(cv::Point_<T> p0_, cv::Point_<T> p1_) : p0(std::move(p0_)), p1(std::move(p1_))
        {            
            auto yd = (p0.y - p1.y);
            auto xd = (p0.x - p1.x);

            if (yd < 0)
                yd = -yd;
            if (xd < 0)
                xd = -xd;

            if (isZero(yd))
                slope = 0;
            else if (isZero(xd))
                slope = std::numeric_limits<double>::max();
            else
                slope = yd / static_cast<double>(xd);
        };

        bool operator==(const Line_<T> &other) const = delete;

        friend std::ostream &operator<<(std::ostream &out, const cv::Point_<T> &point)
        {
            out << "(" << point.x << ", " << point.y << ")";
            return out;
        }

        friend std::ostream &operator<<(std::ostream &out, const Line_<T> &line)
        {
            out << line.p0 << ", " << line.p1 << ": " << line.slope << std::endl;
            return out;
        }

        constexpr bool intersection(const Line_<T> &other, cv::Point_<T> &point) const
        {
            return detail::intersection(*this, other, point);
        }

        constexpr bool parallel(const Line_<T> &other) const
        {
            return detail::parallel(*this, other);
        }

        constexpr double getSegmentDistance(const cv::Point_<T> &point) const
        {
            return detail::getSegmentDistance(*this, point);
        }

        constexpr double getLineDistance(const cv::Point_<T> &point) const
        {
            return detail::getLineDistance(*this, point);
        }

        cv::Point_<T> p0;
        cv::Point_<T> p1;
        double slope;
    };


}