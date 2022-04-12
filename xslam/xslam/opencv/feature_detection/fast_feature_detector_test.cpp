//
// Created by quan on 2021/12/14.
//

#include "xslam/opencv/feature_detection/fast_feature_detector.h"

#include "xslam/opencv/feature_detection/corner_harris.h"
#include "xslam/opencv/utils.h"

#include "gtest/gtest.h"
#include "glog/logging.h"

namespace xslam {
namespace opencv {

TEST(FastFeature, cornerDetect)
{
    std::string filename = GetOpenCVDatasetDirectory() + "/0002_chessboard.jpeg";
    FastFeature demo;
    demo.CornerDetect(filename);
}

} // namespace opencv
} // namespace xslam