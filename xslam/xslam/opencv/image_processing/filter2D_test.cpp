#include "xslam/opencv/image_processing/filter2D.h"

#include "xslam/opencv/utils.h"

#include "gtest/gtest.h"
#include "glog/logging.h"


namespace xslam {
namespace opencv {
namespace image_processing {

TEST(Filter2D, demo)
{
    std::string filename = GetOpenCVDatasetDirectory() + "/0014_roma.jpg";
    Filter2D demo;
    demo.RunDemo(filename);
}

} // namespace image_processing
} // namespace opencv
} // namespace xslam
