#include "xslam/opencv/SIFT.h"
#include "xslam/opencv/utils.h"

#include "gtest/gtest.h"
#include "glog/logging.h"

namespace xslam {
namespace opencv {
    
TEST(SIFT, demo)
{
    
    // 0008_roofs1.jpg
    std::string filename = GetOpenCVDatasetDirectory() + "/0002_chessboard.jpeg"; 
    SIFTFeature demo;
    demo.RunDemo(filename);
}

} // namespace opencv
} // namespace xslam


