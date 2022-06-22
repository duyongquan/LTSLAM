#ifndef XSLAM_OPENCV_GEOMETRY_TRANSFORM_LINE_FITTING_RANSAC_M_EST_H
#define XSLAM_OPENCV_GEOMETRY_TRANSFORM_LINE_FITTING_RANSAC_M_EST_H

#include <opencv2/opencv.hpp>
#include <vector>

#include "ceres/ceres.h"

// Convert a line format, [n_x, n_y, x_0, y_0] to [a, b, c]
// c.f. A line model in OpenCV: n_x * (x - x_0) = n_y * (y - y_0)
#define CONVERT_LINE(line) (cv::Vec3d(line[0], -line[1], -line[0] * line[2] + line[1] * line[3]))

namespace xslam {
namespace opencv {
namespace geometry_transform {

struct GeometricError
{
    GeometricError(const cv::Point2d& pt) : datum(pt) { }

    template<typename T>
    bool operator()(const T* const line, T* residual) const
    {
        residual[0] = (line[0] * T(datum.x) + line[1] * T(datum.y) + line[2]) / sqrt(line[0] * line[0] + line[1] * line[1]);
        return true;
    }

private:
    const cv::Point2d datum;
};

class LineFittingMEst
{
public:
    void RunDemo(const std::string& iamge);

};

} // namespace geometry_transform
} // namespace opencv
} // namespace xslam

#endif // XSLAM_OPENCV_GEOMETRY_TRANSFORM_LINE_FITTING_RANSAC_M_EST_H
