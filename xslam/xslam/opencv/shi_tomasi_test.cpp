//
// Created by quan on 2021/12/14.
//

#include "xslam/opencv/shi_tomasi.h"
#include "xslam/opencv/utils.h"

#include "gtest/gtest.h"
#include "glog/logging.h"

namespace xslam {
namespace opencv {

TEST(ShiTomasi, GoodFeaturesToTrack)
{
    std::string filename = GetOpenCVDatasetDirectory() + "/0002_chessboard.jpeg";
    ShiTomasi demo;
    demo.CornerDetect(filename);
}


} // namespace opencv
} // namespace xslam
