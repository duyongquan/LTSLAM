#include "xslam/opencv/image_basic/clahe.h"

#include "xslam/opencv/utils.h"

#include "gtest/gtest.h"
#include "glog/logging.h"

namespace xslam {
namespace opencv {
namespace image_basic {

TEST(CLAHE, demo)
{
    std::string filename = GetOpenCVDatasetDirectory() + "/0009_CLAHE.jpeg";
    CLAHE demo;
    demo.RunDemo(filename);
}

} // namespace image_basic
} // namespace opencv
} // namespace xslam