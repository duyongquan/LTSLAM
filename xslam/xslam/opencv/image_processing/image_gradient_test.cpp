#include "xslam/opencv/image_processing/image_gradient.h"
#include "xslam/opencv/utils.h"

#include "gtest/gtest.h"
#include "glog/logging.h"


namespace xslam {
namespace opencv {
namespace image_processing {

TEST(Gradient, Sobel)
{
    std::string filename = GetOpenCVDatasetDirectory() + "/0013_parthenon_temple.jpg";
    Gradient demo;
    demo.RunDemo(filename);
}

} // namespace image_processing
} // namespace opencv
} // namespace xslam
