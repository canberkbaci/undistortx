

#include "geometry.h"

#include <cassert>

using namespace geometry;

template <typename T>
void testIntersection()
{
    const auto p0 = cv::Point_<T>(5, 4);
    const auto p1 = cv::Point_<T>(1, 2);

    const auto p2 = cv::Point_<T>(6, 2);
    const auto p3 = cv::Point_<T>(2, 2);

    const auto l0 = Line_<T>(p0, p1);
    const auto l1 = Line_<T>(p2, p3);

    auto p = cv::Point_<T>();

    assert(l0.intersection(l1, p));
    assert(p == cv::Point_<T>(1, 2));
}

template <typename T>
void testParallel()
{
    const auto p0 = cv::Point_<T>(5, 3);
    const auto p1 = cv::Point_<T>(1, 3);

    const auto p2 = cv::Point_<T>(6, 2);
    const auto p3 = cv::Point_<T>(2, 2);

    const auto l0 = Line_<T>(p0, p1);
    const auto l1 = Line_<T>(p2, p3);

    auto p = cv::Point_<T>();

    assert(l0.parallel(l1));
    assert(!l0.intersection(l1, p));
}

template <typename T>
void testGetSegmentDistance()
{
    const auto p0 = cv::Point_<T>(5, 3);
    const auto p1 = cv::Point_<T>(1, 3);
    const auto p2 = cv::Point_<T>(8, 7);

    const auto l0 = Line_<T>(p0, p1);

    assert(l0.getSegmentDistance(p2) == 5.0);
}

template <typename T>
void testGetLineDistance()
{
    const auto p0 = cv::Point_<T>(5, 3);
    const auto p1 = cv::Point_<T>(1, 3);
    const auto p2 = cv::Point_<T>(8, 7);

    const auto l0 = Line_<T>(p0, p1);

    assert(l0.getLineDistance(p2) == 4.0);
}
