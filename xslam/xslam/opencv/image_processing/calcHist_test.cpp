#include "xslam/opencv/image_processing/calcHist.h"

#include "xslam/opencv/utils.h"

#include "gtest/gtest.h"
#include "glog/logging.h"

namespace xslam {
namespace opencv {
namespace image_processing {

TEST(CalcHist, demo)
{
    std::string filename = GetOpenCVDatasetDirectory() + "/0017_pyr.jpg";
    CalcHist demo;
    demo.RunDemo(filename);
}

} // namespace image_processing
} // namespace opencv
} // namespace xslam
