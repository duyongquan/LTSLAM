#ifndef XSLAM_OPENCV_GEOMETRY_TRANSFORM_BUNDLE_ADJUSTMENT_H
#define XSLAM_OPENCV_GEOMETRY_TRANSFORM_BUNDLE_ADJUSTMENT_H

#include "opencv2/opencv.hpp"
#include "glog/logging.h"

#include "ceres/ceres.h"
#include "ceres/rotation.h"

#include <sophus/se3.hpp>
#include <Eigen/Core>
#include <vector>
#include <string>

namespace xslam {
namespace opencv {
namespace geometry_transform {

// Reprojection error for bundle adjustment
struct ReprojectionError
{
    ReprojectionError(const cv::Point2d& _x, double _f, const cv::Point2d& _c) : x(_x), f(_f), c(_c) { }

    template <typename T>
    bool operator()(const T* const camera, const T* const point, T* residuals) const
    {
        // X' = R*X + t
        T X[3];
        ceres::AngleAxisRotatePoint(camera, point, X);
        X[0] += camera[3];
        X[1] += camera[4];
        X[2] += camera[5];

        // x' = K*X'
        T x_p = f * X[0] / X[2] + c.x;
        T y_p = f * X[1] / X[2] + c.y;

        // residual = x - x'
        residuals[0] = T(x.x) - x_p;
        residuals[1] = T(x.y) - y_p;
        return true;
    }

    static ceres::CostFunction* create(const cv::Point2d& _x, double _f, const cv::Point2d& _c)
    {
        return (new ceres::AutoDiffCostFunction<ReprojectionError, 2, 6, 3>(new ReprojectionError(_x, _f, _c)));
    }

private:
    const cv::Point2d x;
    const double f;
    const cv::Point2d c;
};

class BundleAdjustment
{
public:
    void RunDemo(const std::string& filename);
};

} // namespace geometry_transform
} // namespace opencv
} // namespace xslam

#endif // XSLAM_OPENCV_GEOMETRY_TRANSFORM_BUNDLE_ADJUSTMENT_H

