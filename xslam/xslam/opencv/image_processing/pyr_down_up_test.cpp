#include "xslam/opencv/image_processing/pyr_down_up.h"

#include "xslam/opencv/utils.h"

#include "gtest/gtest.h"
#include "glog/logging.h"

namespace xslam {
namespace opencv {
namespace image_processing {

TEST(Threshold, demo)
{
    std::string filename = GetOpenCVDatasetDirectory() + "/0017_pyr.jpg";
    PyrDownUp demo;
    demo.RunDemo(filename);
}

} // namespace image_processing
} // namespace opencv
} // namespace xslam
