#ifndef XSLAM_OPENCV_GEOMETRY_TRANSFORM_LINE_FITTING_RANSAC_H
#define XSLAM_OPENCV_GEOMETRY_TRANSFORM_LINE_FITTING_RANSAC_H

#include <opencv2/opencv.hpp>
#include <vector>

// Convert a line format, [n_x, n_y, x_0, y_0] to [a, b, c]
// c.f. A line model in OpenCV: n_x * (x - x_0) = n_y * (y - y_0)
#define CONVERT_LINE(line) (cv::Vec3d(line[0], -line[1], -line[0] * line[2] + line[1] * line[3]))

namespace xslam {
namespace opencv {
namespace geometry_transform {

class LineFittingRANSAC
{
public:
    void RunDemo(const std::string& iamge);

};

} // namespace geometry_transform
} // namespace opencv
} // namespace xslam

#endif // XSLAM_OPENCV_GEOMETRY_TRANSFORM_LINE_FITTING_RANSAC_H
