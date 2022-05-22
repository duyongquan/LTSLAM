#include "xslam/opencv/video_analysis/cam_shift.h"
#include "xslam/opencv/utils.h"

#include "gtest/gtest.h"
#include "glog/logging.h"

namespace xslam {
namespace opencv {
namespace video_analysis {

TEST(Camshift, demo)
{
    std::string filename = GetOpenCVDatasetDirectory() + "/cup.mp4";
    Camshift demo;
    demo.RunDemo(filename);
}

} // namespace video_analysis
} // namespace opencv
} // namespace xslam
