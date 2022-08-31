
#include "undistortion.h"

#include <ceres/ceres.h>

using Vec6 = Eigen::Matrix<double, 7, 1>;

namespace undistortion
{
    namespace detail
    {
        cv::Point2d undistort(const cv::Point2d &pd, const DistortionCoefficients &kd)
        {
            const auto dx = pd.x - kd.cx;
            const auto dy = pd.y - kd.cy;
            const auto r2 = pow(dx, 2) + pow(dy, 2);
            const auto f = 1 + kd.k1 * r2 + kd.k2 * pow(r2, 2) + kd.k3 * pow(r2, 3);

            cv::Point2d pu;
            pu.x = kd.cx + (f * dx) + (kd.p2 * (r2 + 2 * pow(dx, 2))) + (2 * kd.p1 * dx * dy);
            pu.y = kd.cy + (f * dy) + (kd.p1 * (r2 + 2 * pow(dy, 2))) + (2 * kd.p2 * dx * dy);
            return pu;
        }

        std::vector<cv::Point2d> undistort(const std::vector<cv::Point2d> &points, const DistortionCoefficients &kd)
        {
            std::vector<cv::Point2d> pu;
            for (const auto &pd : points)
            {
                pu.push_back(undistort(pd, kd));
            }
            return pu;
        }

        geometry::Line_<double> fitLine(const std::vector<cv::Point2d> &points)
        {
            cv::Vec4f line;
            cv::fitLine(points, line, cv::DIST_L2, 0, 0.1, 0.1);

            double vx = line[0];
            double vy = line[1];
            double x = line[2];
            double y = line[3];

            cv::Point2d p1(INT_MAX, ((INT_MAX - x) * (vy / vx)) + y);
            cv::Point2d p2(0, (-x * vy / vx) + y);

            return geometry::Line_<double>(p1, p2);
        }

        double calculateError(const std::vector<cv::Point2d> &points)
        {
            const auto line = fitLine(points);

            std::vector<double> errors;
            errors.reserve(points.size());
            for (const auto &p : points)
            {
                const auto d = line.getLineDistance(p);
                errors.push_back(pow(d, 2));
            }
            return sqrt(cv::sum(errors)[0]);
        }
    }

    bool UndistortionCostFunctor::operator()(const double *coeffs, double *residual) const
    {
        Vec6 kd_(coeffs);

        DistortionCoefficients kd;
        kd.k1 = kd_(0);
        kd.k2 = kd_(1);
        kd.p1 = kd_(2);
        kd.p2 = kd_(3);
        kd.k3 = kd_(4);
        kd.cx = kd_(5);
        kd.cy = kd_(6);

        std::vector<cv::Point2d> pu = detail::undistort(pd, kd);
        residual[0] = detail::calculateError(pu);

        return true;
    }

    void Undistortion::calculateDistortionCoefficients(const std::vector<std::vector<cv::Point2d>> &points)
    {
        // initial guess
        Vec6 kd_vec;
        kd_vec(0) = 0;
        kd_vec(1) = 0;
        kd_vec(2) = 0;
        kd_vec(3) = 0;
        kd_vec(4) = 0;
        kd_vec(5) = resolution.width * .5;
        kd_vec(6) = resolution.height * .5;

        // define problem
        // add residuals for each curve with its cost for each kd_vec param set
        ceres::Problem problem;
        for (const auto &pd : points)
        {
            UndistortionCostFunctor *func = new UndistortionCostFunctor(pd);
            problem.AddResidualBlock(
                new ceres::NumericDiffCostFunction<UndistortionCostFunctor, ceres::CENTRAL, 1, 7>(func),
                NULL, kd_vec.data());
        }

        // limit sensor center cx cy to a small region
        // problem.SetParameterLowerBound(kd_vec.data(), 5, resolution.width * .5 - 5);
        // problem.SetParameterUpperBound(kd_vec.data(), 5, resolution.width * .5 + 5);
        // problem.SetParameterLowerBound(kd_vec.data(), 5, resolution.height * .5 - 5);
        // problem.SetParameterUpperBound(kd_vec.data(), 5, resolution.height * .5 + 5);

        ceres::Solver::Options options;
        options.minimizer_type = ceres::TRUST_REGION;
        options.use_nonmonotonic_steps = 1;
        options.max_consecutive_nonmonotonic_steps = 1e3;
        options.function_tolerance = 0;
        options.parameter_tolerance = 0;
        options.gradient_tolerance = 0;
        options.max_num_consecutive_invalid_steps = 1e3;
        options.linear_solver_type = ceres::SPARSE_SCHUR;
        options.max_num_iterations = 1e6;
        options.max_solver_time_in_seconds = 20;
        options.update_state_every_iteration = true;
        options.logging_type = ceres::SILENT;

        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        if (!summary.IsSolutionUsable())
        {
            throw error::UndistortionError::CERES_ERROR;
        }

        std::cout << "Summary:" << std::endl;
        std::cout << summary.FullReport() << std::endl;
        std::cout << "Final refined params:" << std::endl;
        std::cout << kd_vec << std::endl;
        std::cout << "Final cost: " << summary.final_cost << std::endl;

        kd.k1 = kd_vec(0);
        kd.k2 = kd_vec(1);
        kd.p1 = kd_vec(2);
        kd.p2 = kd_vec(3);
        kd.k3 = kd_vec(4);
        kd.cx = kd_vec(5);
        kd.cy = kd_vec(6);
    }

    void Undistortion::calculateDistortionMapping(const DistortionCoefficients &kd)
    {
        cv::Point2d tl = detail::undistort(cv::Point2d(0, 0), kd);
        cv::Point2d tr = detail::undistort(cv::Point2d(resolution.width, 0), kd);
        cv::Point2d bl = detail::undistort(cv::Point2d(0, resolution.height), kd);
        cv::Point2d br = detail::undistort(cv::Point2d(resolution.width, resolution.height), kd);

        map.off_x = 0;
        map.off_y = 0;
        map.ext_x = 0;
        map.ext_y = 0;

        if (tl.x <= bl.x && tl.x < 0)
        {
            map.off_x = 0 - tl.x;
            map.ext_x = 0 - tl.x;
        }
        else if (bl.x < tl.x && bl.x < 0)
        {
            map.off_x = 0 - bl.x;
            map.ext_x = 0 - bl.x;
        }
        if (tl.y <= tr.y && tl.y < 0)
        {
            map.off_y = map.off_y + (0 - tl.y);
            map.ext_y = 0 - tl.y;
        }
        else if (tr.y < tl.y && tr.y < 0)
        {
            map.off_y = map.off_y + (0 - tr.y);
            map.ext_y = 0 - tr.y;
        }

        if (tr.x >= br.x && tr.x >= resolution.width)
            map.ext_x = map.ext_x + (tr.x - resolution.width + 1);
        else if (br.x > tr.x && br.x >= resolution.width)
            map.ext_x = map.ext_x + (br.x - resolution.width + 1);
        if (bl.y >= br.y && bl.y >= resolution.height)
            map.ext_y = map.ext_y + (bl.y - resolution.height + 1);
        else if (br.y > bl.y && br.y >= resolution.height)
            map.ext_y = map.ext_y + (br.y - resolution.height + 1);

        // calculate distortion lookup tables
        map.map_x = cv::Mat::zeros(resolution.height + map.ext_y, resolution.width + map.ext_x, CV_32FC1);
        map.map_y = cv::Mat::zeros(resolution.height + map.ext_y, resolution.width + map.ext_x, CV_32FC1);

        for (auto x = -map.off_x; x < resolution.width + map.ext_x; x++)
        {
            for (auto y = -map.off_y; y < resolution.height + map.ext_y; y++)
            {
                // calculate undistorted point
                cv::Point2d pd(x, y);
                cv::Point2d pu = detail::undistort(pd, kd);

                // if undistorted point is in map image range, insert <undistorted point, distorted point> correspondence to map
                if (0 <= floor(pu.x + map.off_x) && ceil(pu.x + map.off_x) < map.map_x.cols &&
                    0 <= floor(pu.y + map.off_y) && ceil(pu.y + map.off_y) < map.map_x.rows)
                {
                    map.map_x.at<float>(ceil(pu.y + map.off_y), ceil(pu.x + map.off_x)) = x;
                    map.map_y.at<float>(ceil(pu.y + map.off_y), ceil(pu.x + map.off_x)) = y;

                    map.map_x.at<float>(floor(pu.y + map.off_y), floor(pu.x + map.off_x)) = x;
                    map.map_y.at<float>(floor(pu.y + map.off_y), floor(pu.x + map.off_x)) = y;

                    map.map_x.at<float>(ceil(pu.y + map.off_y), floor(pu.x + map.off_x)) = x;
                    map.map_y.at<float>(ceil(pu.y + map.off_y), floor(pu.x + map.off_x)) = y;

                    map.map_x.at<float>(floor(pu.y + map.off_y), ceil(pu.x + map.off_x)) = x;
                    map.map_y.at<float>(floor(pu.y + map.off_y), ceil(pu.x + map.off_x)) = y;
                }
            }
        }
    }

    void Undistortion::warp(const cv::Mat &image, const DistortionMapping &map)
    {
        if (cv::Size(image.cols, image.rows) != resolution)
        {
            throw error::UndistortionError::IMAGE_SIZE_ERROR;
        }

        cv::remap(image, warped, map.map_x, map.map_y, cv::INTER_LINEAR);
        // auto x = std::max(0, warped.cols / 2 - resolution.width);
        // auto y = std::max(0, warped.rows / 2 - resolution.height);
        // cv::Rect bbox = cv::Rect(x, y, 2 * resolution.width, 2 * resolution.height);
        // warped = warped(bbox);
    }

    bool Undistortion::undistort(const cv::Mat &input, const DistortionCoefficients &kd_, cv::Mat &output)
    {
        kd = kd_;
        auto status = false;
        resolution = cv::Size(input.cols, input.rows);

        try
        {
            calculateDistortionMapping(kd);
            warp(input, map);

            output = warped.clone();
            status = true;
        }
        catch (const error::UndistortionError &error)
        {
            std::cerr << "Undistortion error: " << error::ErrorDescription.at(error) << std::endl;
        }

        return status;
    }

    bool Undistortion::undistort(const cv::Mat &input, const std::vector<std::vector<cv::Point2d>> &pd, cv::Mat &output)
    {
        auto status = false;
        resolution = cv::Size(input.cols, input.rows);

        try
        {
            calculateDistortionCoefficients(pd);
            calculateDistortionMapping(kd);
            warp(input, map);

            output = warped.clone();
            status = true;
        }
        catch (const error::UndistortionError &error)
        {
            std::cerr << "Undistortion error: " << error::ErrorDescription.at(error) << std::endl;
        }

        return status;
    }

}