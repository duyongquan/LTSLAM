#include "xslam/opencv/image_processing/hough_lines.h"

#include "xslam/opencv/utils.h"

#include "gtest/gtest.h"
#include "glog/logging.h"

namespace xslam {
namespace opencv {
namespace image_processing {

TEST(HoughLines, demo)
{
    std::string filename = GetOpenCVDatasetDirectory() + "/0019_hough_line_2.png";
    HoughLines demo;
    demo.RunDemo(filename);
}

} // namespace image_processing
} // namespace opencv
} // namespace xslam
