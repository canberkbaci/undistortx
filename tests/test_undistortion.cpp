#include "undistortion.h"

#include <cassert>

void testUndistort1()
{
    std::vector<cv::Point2d> curve_1;
    curve_1.push_back(cv::Point2d(49, 637));
    curve_1.push_back(cv::Point2d(193, 646));
    curve_1.push_back(cv::Point2d(363, 653));
    curve_1.push_back(cv::Point2d(478, 655));
    curve_1.push_back(cv::Point2d(717, 656));
    curve_1.push_back(cv::Point2d(889, 654));
    curve_1.push_back(cv::Point2d(1068, 643));
    curve_1.push_back(cv::Point2d(1223, 631));

    std::vector<cv::Point2d> curve_2;
    curve_2.push_back(cv::Point2d(52, 516));
    curve_2.push_back(cv::Point2d(178, 409));
    curve_2.push_back(cv::Point2d(251, 351));
    curve_2.push_back(cv::Point2d(338, 282));
    curve_2.push_back(cv::Point2d(400, 234));
    curve_2.push_back(cv::Point2d(452, 195));

    std::vector<cv::Point2d> curve_3;
    curve_3.push_back(cv::Point2d(1259, 446));
    curve_3.push_back(cv::Point2d(1191, 398));
    curve_3.push_back(cv::Point2d(1101, 336));
    curve_3.push_back(cv::Point2d(1010, 275));
    curve_3.push_back(cv::Point2d(938, 229));
    curve_3.push_back(cv::Point2d(926, 229));
    curve_3.push_back(cv::Point2d(869, 187));

    std::vector<std::vector<cv::Point2d>> pointsOnCurves;
    pointsOnCurves.push_back(curve_1);
    pointsOnCurves.push_back(curve_2);
    pointsOnCurves.push_back(curve_3);

    cv::Mat input = cv::imread("../tests/data/1.jpg", cv::IMREAD_COLOR);
    undistortion::Undistortion worker;
    worker.undistort(input, pointsOnCurves);
    const DistortionCoefficients actual = worker.getDistortionCoefficients();
    const DistortionCoefficients expected{2.19668e-16, 1.53243e-18, -1.14718e-13, 5.91584e-14, 1.17589e-18, 640, 360};
    assert(actual == expected);
}

void testUndistort2()
{
    std::vector<cv::Point2d> curve_1;
    curve_1.push_back(cv::Point2d(409, 24));
    curve_1.push_back(cv::Point2d(435, 64));
    curve_1.push_back(cv::Point2d(459, 105));
    curve_1.push_back(cv::Point2d(481, 154));
    curve_1.push_back(cv::Point2d(505, 227));
    curve_1.push_back(cv::Point2d(515, 288));

    std::vector<cv::Point2d> curve_2;
    curve_2.push_back(cv::Point2d(119, 168));
    curve_2.push_back(cv::Point2d(179, 165));
    curve_2.push_back(cv::Point2d(281, 164));
    curve_2.push_back(cv::Point2d(426, 168));
    curve_2.push_back(cv::Point2d(529, 177));

    std::vector<cv::Point2d> curve_3;
    curve_3.push_back(cv::Point2d(137, 36));
    curve_3.push_back(cv::Point2d(189, 23));
    curve_3.push_back(cv::Point2d(262, 13));
    curve_3.push_back(cv::Point2d(349, 15));
    curve_3.push_back(cv::Point2d(537, 72));

    std::vector<cv::Point2d> curve_4;
    curve_4.push_back(cv::Point2d(78, 330));
    curve_4.push_back(cv::Point2d(152, 359));
    curve_4.push_back(cv::Point2d(244, 377));
    curve_4.push_back(cv::Point2d(376, 375));
    curve_4.push_back(cv::Point2d(507, 341));
    curve_4.push_back(cv::Point2d(564, 310));

    std::vector<std::vector<cv::Point2d>> pointsOnCurves;
    pointsOnCurves.push_back(curve_1);
    pointsOnCurves.push_back(curve_2);
    pointsOnCurves.push_back(curve_3);
    pointsOnCurves.push_back(curve_4);

    cv::Mat input = cv::imread("../tests/data/2.jpg", cv::IMREAD_COLOR);
    undistortion::Undistortion worker;
    worker.undistort(input, pointsOnCurves);
    const DistortionCoefficients actual = worker.getDistortionCoefficients();
    const DistortionCoefficients expected{3.79598e-08, 5.04935e-11, -3.57413e-05, -3.94738e-07, 8.26014e-16, 300.18, 200.356};

    assert(actual == expected);
}