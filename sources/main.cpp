
#include "undistortion.h"

#include <string>

#include <opencv2/opencv.hpp>

namespace detail
{
    const std::string pointsOnCurvesStr = "pointsOnCurves";
    const std::string distortionCoefficientsStr = "distortionCoefficients";
}

std::vector<std::vector<cv::Point2d>> readPoints(const cv::FileStorage &fs)
{
    cv::FileNode pointsOnCurves = fs[detail::pointsOnCurvesStr];
    std::vector<std::vector<cv::Point2d>> pds;
    for (cv::FileNodeIterator itData = pointsOnCurves.begin(); itData != pointsOnCurves.end(); ++itData)
    {
        std::vector<cv::Point2d> pd;
        cv::FileNode pts = *itData;
        for (cv::FileNodeIterator itPts = pts.begin(); itPts != pts.end(); ++itPts)
        {
            cv::FileNode pt = *itPts;
            cv::Point2d point;
            cv::FileNodeIterator itPt = pt.begin();
            point.x = *itPt;
            ++itPt;
            point.y = *itPt;
            pd.push_back(point);
        }
        pds.push_back(pd);
    }

    return pds;
}

DistortionCoefficients readDistortionCoefficients(const cv::FileStorage &fs)
{
    cv::FileNode distortionCoefficients = fs[detail::distortionCoefficientsStr];
    cv::Mat kd;
    distortionCoefficients >> kd;
    DistortionCoefficients kd_{kd.at<double>(0), kd.at<double>(1), kd.at<double>(2), kd.at<double>(3), kd.at<double>(4), kd.at<double>(5), kd.at<double>(6)};
    return kd_;
}

int main(int argc, char *argv[])
{
    bool status = false;

    std::string imageFilename(argv[1]);
    std::string yamlFilename(argv[2]);

    const auto image = cv::imread(imageFilename, cv::IMREAD_COLOR);
    cv::Mat imageUndistorted;

    cv::FileStorage fs(yamlFilename, cv::FileStorage::READ);

    cv::FileNode distortionCoefficients = fs[detail::distortionCoefficientsStr];
    if (distortionCoefficients.empty())
    {
        std::cout << "Undistortion by estimating coefficients..." << std::endl;

        const auto pds = readPoints(fs);
        undistortion::Undistortion worker;
        status = worker.undistort(image, pds, imageUndistorted);
    }
    else
    {
        std::cout << "Undistortion with known coefficients..." << std::endl;

        const auto kd = readDistortionCoefficients(fs);
        undistortion::Undistortion worker;
        status = worker.undistort(image, kd, imageUndistorted);
    }

    if (status)
    {
        cv::imshow("original", image);
        cv::imshow("undistorted", imageUndistorted);
        cv::waitKey(0);

        size_t lastindex = imageFilename.find_last_of("."); 
        std::string basename = imageFilename.substr(0, lastindex); 

        cv::imwrite(basename + "_undist.jpg", imageUndistorted);
    }

    return 0;
}