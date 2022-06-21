#include "xslam/opencv/geometry_transform/camera_calibration.h"
#include "xslam/opencv/utils.h"

#include "gtest/gtest.h"
#include "glog/logging.h"

namespace xslam {
namespace opencv {
namespace geometry_transform {

TEST(CameraCalibration, camera_calibration)
{
    std::string avi = GetOpenCVDatasetDirectory() + "/chessboard.avi";
    CameraCalibration demo;
    demo.RunDemo(avi);
}

} // namespace geometry_transform
} // namespace opencv
} // namespace xslam
