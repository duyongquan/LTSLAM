#include "xslam/opencv/image_processing/matchTemplate.h"

#include "xslam/opencv/utils.h"

#include "gtest/gtest.h"
#include "glog/logging.h"

namespace xslam {
namespace opencv {
namespace image_processing {

TEST(MatchTemplate, demo)
{
    std::string filename = GetOpenCVDatasetDirectory() + "/0018_monkey.jpg";
    std::string template_image = GetOpenCVDatasetDirectory() + "/0018_monkey_template.png";
    MatchTemplate demo;
    demo.RunDemo(filename, template_image);
}

} // namespace image_processing
} // namespace opencv
} // namespace xslam
