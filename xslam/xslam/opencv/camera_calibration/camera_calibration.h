#ifndef XSLAM_OPENCV_CAMERA_CALIBRATION_CAMERA_CALIBRATION_H
#define XSLAM_OPENCV_CAMERA_CALIBRATION_CAMERA_CALIBRATION_H

#include "opencv2/opencv.hpp"
#include "glog/logging.h"

#include <sophus/se3.hpp>
#include <Eigen/Core>
#include <vector>
#include <string>

namespace xslam {
namespace opencv {
namespace camera_calibration {

class CameraCalibration
{
public:
    void RunDemo(const std::string& filename);
};

} // namespace camera_calibration
} // namespace opencv
} // namespace xslam

#endif // XSLAM_OPENCV_CAMERA_CALIBRATION_CAMERA_CALIBRATION_H

