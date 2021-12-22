//
// Created by quan on 2021/12/14.
//

#include "xslam/opencv/corner_harris.h"
#include "xslam/opencv/utils.h"

#include "gtest/gtest.h"
#include "glog/logging.h"

namespace slam {
namespace opencv {

TEST(CornerHarris, cornerDetect)
{
    std::string filename = GetOpenCVDatasetDirectory() + "/0002_chessboard.jpeg";
    CornerHarris demo;
    demo.CornerDetect(filename);
}

} // namespace opencv
} // namespace slam