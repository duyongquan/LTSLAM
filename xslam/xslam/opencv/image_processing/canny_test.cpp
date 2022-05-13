#include "xslam/opencv/image_processing/canny.h"
#include "xslam/opencv/utils.h"

#include "gtest/gtest.h"
#include "glog/logging.h"

namespace xslam {
namespace opencv {
namespace image_processing {

TEST(Canny, shape)
{
    LOG(INFO) << "Run Canny demos ...";
    
    // OpenCV
    std::string filename = GetOpenCVDatasetDirectory() + "/0011_canny.jpg";
    Canny demo;
    demo.RunDemo(filename);
}

} // namespace image_processing
} // namespace opencv
} // namespace xslam
