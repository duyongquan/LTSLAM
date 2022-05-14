#include "xslam/opencv/image_processing/cvtColor.h"

#include "xslam/opencv/utils.h"

#include "gtest/gtest.h"
#include "glog/logging.h"


namespace xslam {
namespace opencv {
namespace image_processing {

TEST(CvtColor, demo)
{
    std::string filename = GetOpenCVDatasetDirectory() + "/0015_gudian.jpg";
    CvtColor demo;
    demo.RunDemo(filename);
}

} // namespace image_processing
} // namespace opencv
} // namespace xslam
