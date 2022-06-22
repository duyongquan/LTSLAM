#ifndef XSLAM_OPENCV_GEOMETRY_TRANSFORM_POSE_ESTIMATION_H
#define XSLAM_OPENCV_GEOMETRY_TRANSFORM_POSE_ESTIMATION_H

#include "opencv2/opencv.hpp"
#include "glog/logging.h"

#include <sophus/se3.hpp>
#include <Eigen/Core>
#include <vector>
#include <string>

namespace xslam {
namespace opencv {
namespace geometry_transform {

class PoseEstimation
{
public:
    void RunDemo1(const std::string& vedio, const std::string& image);
    void RunDemo2(const std::string& vedio, const std::string& image);
    void RunDemo3(const std::string& vedio, const std::string& image);
};

} // namespace geometry_transform
} // namespace opencv
} // namespace xslam

#endif // XSLAM_OPENCV_GEOMETRY_TRANSFORM_POSE_ESTIMATION_H

