#ifndef XSLAM_OPENCV_GEOMETRY_TRANSFORM_CAMERA_CALIBRATION_H
#define XSLAM_OPENCV_GEOMETRY_TRANSFORM_CAMERA_CALIBRATION_H

#include "opencv2/opencv.hpp"
#include "glog/logging.h"

#include <sophus/se3.hpp>
#include <Eigen/Core>
#include <vector>
#include <string>

namespace xslam {
namespace opencv {
namespace geometry_transform {

class CameraCalibration
{
public:
    void RunDemo(const std::string& filename);
};

} // namespace geometry_transform
} // namespace opencv
} // namespace xslam

#endif // XSLAM_OPENCV_GEOMETRY_TRANSFORM_CAMERA_CALIBRATION_H

