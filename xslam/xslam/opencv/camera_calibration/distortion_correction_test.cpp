#include "xslam/opencv/camera_calibration/distortion_correction.h"
#include "xslam/opencv/utils.h"

#include "gtest/gtest.h"
#include "glog/logging.h"

namespace xslam {
namespace opencv {
namespace camera_calibration {

TEST(DistortionCorrection, distortion_correction)
{
    std::string avi = GetOpenCVDatasetDirectory() + "/chessboard.avi";
    DistortionCorrection demo;
    demo.RunDemo(avi);
}

} // namespace camera_calibration
} // namespace opencv
} // namespace xslam