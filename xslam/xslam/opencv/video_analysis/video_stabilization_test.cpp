#include "xslam/opencv/video_analysis/video_stabilization.h"
#include "xslam/opencv/utils.h"

#include "gtest/gtest.h"
#include "glog/logging.h"

namespace xslam {
namespace opencv {
namespace video_analysis {

TEST(VideoStabilization, video_stabilization)
{
    std::string filename = GetOpenCVDatasetDirectory() + "/traffic.avi";
    VideoStabilization demo;
    demo.RunDemo(filename);
}

} // namespace video_analysis
} // namespace opencv
} // namespace xslam
