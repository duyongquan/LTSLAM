#include "xslam/opencv/video_analysis/optical_flow.h"
#include "xslam/opencv/utils.h"

#include "gtest/gtest.h"
#include "glog/logging.h"

namespace xslam {
namespace opencv {
namespace video_analysis {

TEST(OpticalFlow, demo)
{
    std::string filename = GetOpenCVDatasetDirectory() + "/slow_traffic_small.mp4";
    OpticalFlow demo;
    demo.RunDemo(filename);
}

} // namespace video_analysis
} // namespace opencv
} // namespace xslam
