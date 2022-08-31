#include "test_geometry.cpp"
#include "test_undistortion.cpp"

int main()
{
    testIntersection<int>();
    testIntersection<double>();

    testParallel<int>();
    testParallel<double>();

    testGetSegmentDistance<int>();
    testGetSegmentDistance<double>();

    testGetLineDistance<int>();
    testGetLineDistance<double>();

    testUndistort1();
    testUndistort2();

    std::cout << "TEST OK" << std::endl;
}