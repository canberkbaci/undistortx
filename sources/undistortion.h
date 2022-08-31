
#pragma once

#include "geometry.h"
#include "types.h"

#include <array>
#include <iostream>
#include <string>
#include <cmath>
#include <chrono>
#include <sstream>
#include <fstream>
#include <chrono>
#include <limits>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <opencv2/opencv.hpp>

namespace undistortion
{
    namespace error
    {
        enum UndistortionError
        {
            CERES_ERROR = 0,
            IMAGE_SIZE_ERROR = 1
        };

        const std::map<UndistortionError, std::string> ErrorDescription{{CERES_ERROR, "Error while solving least squares problem"}, {IMAGE_SIZE_ERROR, "Incorrect image size"}};
    }

    namespace detail
    {
        static cv::Point2d undistort(const cv::Point2d &pd, const DistortionCoefficients &dc);
        static std::vector<cv::Point2d> undistort(const std::vector<cv::Point2d> &pds, const DistortionCoefficients &dc);

        static geometry::Line_<double> fitLine(const std::vector<cv::Point2d> &points);
        static double calculateError(const std::vector<cv::Point2d> &points);
    }

    class UndistortionCostFunctor
    {
    public:
        UndistortionCostFunctor(const std::vector<cv::Point2d> &pd_) : pd(pd_) {}

        bool operator()(const double *coeffs, double *residual) const;

    private:
        const std::vector<cv::Point2d> pd;
    };

    class Undistortion
    {
    public:
        bool undistort(const cv::Mat &input, const DistortionCoefficients& kd_, cv::Mat &output);
        bool undistort(const cv::Mat &input, const std::vector<std::vector<cv::Point2d>> &pd, cv::Mat &output);

        DistortionCoefficients getDistortionCoefficients() { return kd; };

    private:
        void calculateDistortionCoefficients(const std::vector<std::vector<cv::Point2d>> &points);
        void calculateDistortionMapping(const DistortionCoefficients &kd);
        void warp(const cv::Mat &image, const DistortionMapping &map);

        cv::Size resolution;
        DistortionCoefficients kd;
        DistortionMapping map;
        cv::Mat warped;
    };
}