#include "xslam/opencv/video_analysis/mean_shift.h"
#include "xslam/opencv/utils.h"

#include "gtest/gtest.h"
#include "glog/logging.h"

namespace xslam {
namespace opencv {
namespace video_analysis {

TEST(Meanshift, demo)
{
    std::string filename = GetOpenCVDatasetDirectory() + "/slow_traffic_small.mp4";
    Meanshift demo;
    demo.RunDemo(filename);
}

} // namespace video_analysis
} // namespace opencv
} // namespace xslam
