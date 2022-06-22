#include "xslam/opencv/camera_calibration/perspective_correction.h"
#include "xslam/opencv/utils.h"

#include "gtest/gtest.h"
#include "glog/logging.h"

namespace xslam {
namespace opencv {
namespace camera_calibration {

TEST(PerspectiveCorrection, perspective_correction)
{
    std::string avi = GetOpenCVDatasetDirectory() + "/0027_sunglok_desk.jpg";
    PerspectiveCorrection demo;
    demo.RunDemo(avi);
}

} // namespace camera_calibration
} // namespace opencv
} // namespace xslam