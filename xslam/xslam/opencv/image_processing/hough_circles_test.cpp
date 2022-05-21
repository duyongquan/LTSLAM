#include "xslam/opencv/image_processing/hough_circles.h"

#include "xslam/opencv/utils.h"

#include "gtest/gtest.h"
#include "glog/logging.h"

namespace xslam {
namespace opencv {
namespace image_processing {

TEST(HoughCircles, demo)
{
    std::string filename = GetOpenCVDatasetDirectory() + "/0020_hough_circle_1.jpeg";
    HoughCircles demo;
    demo.RunDemo(filename);
}

} // namespace image_processing
} // namespace opencv
} // namespace xslam
