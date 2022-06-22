#ifndef XSLAM_OPENCV_GEOMETRY_TRANSFORM_TRIANGULATION_1_H
#define XSLAM_OPENCV_GEOMETRY_TRANSFORM_TRIANGULATION_1_H

#include "opencv2/opencv.hpp"
#include "glog/logging.h"

#include <sophus/se3.hpp>
#include <Eigen/Core>
#include <vector>
#include <string>

namespace xslam {
namespace opencv {
namespace geometry_transform {

class Triangulation
{
public:
    void RunDemo(const std::string& image1, const std::string& image2);
};

} // namespace geometry_transform
} // namespace opencv
} // namespace xslam

#endif // XSLAM_OPENCV_GEOMETRY_TRANSFORM_TRIANGULATION_1_H

