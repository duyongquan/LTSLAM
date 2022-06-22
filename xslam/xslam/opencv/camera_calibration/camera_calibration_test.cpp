#include "xslam/opencv/camera_calibration/camera_calibration.h"
#include "xslam/opencv/utils.h"

#include "gtest/gtest.h"
#include "glog/logging.h"

namespace xslam {
namespace opencv {
namespace camera_calibration {

TEST(CameraCalibration, camera_calibration)
{
    std::string avi = GetOpenCVDatasetDirectory() + "/chessboard.avi";
    CameraCalibration demo;
    demo.RunDemo(avi);
}

} // namespace camera_calibration
} // namespace opencv
} // namespace xslam
