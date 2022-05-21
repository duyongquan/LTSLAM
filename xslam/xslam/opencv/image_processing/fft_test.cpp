#include "xslam/opencv/image_processing/fft.h"

#include "xslam/opencv/utils.h"

#include "gtest/gtest.h"
#include "glog/logging.h"

namespace xslam {
namespace opencv {
namespace image_processing {

TEST(FFT, demo)
{
    std::string filename = GetOpenCVDatasetDirectory() + "/0021_fft_1.jpg";
    FFT demo;
    demo.RunDemo(filename);
}

} // namespace image_processing
} // namespace opencv
} // namespace xslam
