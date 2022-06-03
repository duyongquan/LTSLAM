#include "xslam/opencv/video_analysis/build_optical_flow_pyramid.h"
#include "xslam/opencv/utils.h"

#include "gtest/gtest.h"
#include "glog/logging.h"

namespace xslam {
namespace opencv {
namespace video_analysis {

TEST(BuildOpticalFlowPyramid, demo)
{
    std::string filename = GetOpenCVDatasetDirectory() + "/0002_dota2.avi";
    BuildOpticalFlowPyramid demo;
    demo.RunDemo(filename);
}

} // namespace video_analysis
} // namespace opencv
} // namespace xslam
