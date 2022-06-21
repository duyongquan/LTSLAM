#include "xslam/opencv/feature_detection/image_stitching.h"
#include "xslam/opencv/utils.h"

#include "gtest/gtest.h"
#include "glog/logging.h"

namespace xslam {
namespace opencv {
namespace feature_detection {

TEST(ImageStitching, cornerDetect)
{
    std::string image1 = GetOpenCVDatasetDirectory() + "/0025_hill01.jpg";
    std::string image2 = GetOpenCVDatasetDirectory() + "/0025_hill02.jpg";

    ImageStitching demo;
    demo.RunDemo(image1, image2);
}

} // namespace feature_detection
} // namespace opencv
} // namespace xslam


