#include "xslam/opencv/BRIEF.h"
#include "xslam/opencv/utils.h"

#include "gtest/gtest.h"
#include "glog/logging.h"

namespace xslam {
namespace opencv {

TEST(CornerHarris, cornerDetect)
{
    std::string filename = GetOpenCVDatasetDirectory() + "/0002_chessboard.jpeg";
    BRIEF demo;
    // demo.CornerDetect(filename);
}

} // namespace opencv
} // namespace xslam