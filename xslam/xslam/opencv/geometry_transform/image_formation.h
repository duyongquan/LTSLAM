#ifndef XSLAM_OPENCV_GEOMETRY_TRANSFORM_IMAGE_FORMATION_H
#define XSLAM_OPENCV_GEOMETRY_TRANSFORM_IMAGE_FORMATION_H

#include "opencv2/opencv.hpp"
#include "glog/logging.h"

#include <sophus/se3.hpp>
#include <Eigen/Core>
#include <vector>
#include <string>

namespace xslam {
namespace opencv {
namespace geometry_transform {

#define Rx(rx)      (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, cos(rx), -sin(rx), 0, sin(rx), cos(rx))
#define Ry(ry)      (cv::Mat_<double>(3, 3) << cos(ry), 0, sin(ry), 0, 1, 0, -sin(ry), 0, cos(ry))
#define Rz(rz)      (cv::Mat_<double>(3, 3) << cos(rz), -sin(rz), 0, sin(rz), cos(rz), 0, 0, 0, 1)

class ImageFormation
{
public:
    void RunDemo(const std::string& filename);
};

} // namespace geometry_transform
} // namespace opencv
} // namespace xslam

#endif // XSLAM_OPENCV_GEOMETRY_TRANSFORM_IMAGE_FORMATION_H

