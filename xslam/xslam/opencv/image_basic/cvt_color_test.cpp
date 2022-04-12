#include "xslam/opencv/image_basic/cvt_color.h"

#include "xslam/opencv/utils.h"

#include "gtest/gtest.h"
#include "glog/logging.h"

namespace xslam {
namespace opencv {
namespace image_basic {

TEST(CvtColor, demo)
{
    std::string filename = GetOpenCVDatasetDirectory() + "/0010_cvt_color.jpeg";
    CvtColor demo;
    demo.RunDemo(filename);
}

} // namespace image_basic
} // namespace opencv
} // namespace xslam