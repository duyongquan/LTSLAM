#include "xslam/opencv/image_processing/watershed.h"

#include "xslam/opencv/utils.h"

#include "gtest/gtest.h"
#include "glog/logging.h"

namespace xslam {
namespace opencv {
namespace image_processing {

TEST(Watershed, demo)
{
    std::string filename = GetOpenCVDatasetDirectory() + "/0023_cards.png";
    Watershed demo;
    demo.RunDemo(filename);
}

} // namespace image_processing
} // namespace opencv
} // namespace xslam
