//
// Created by quan on 2021/12/20.
//

#include "xslam/opencv/pose_estimation_2d2d.h"
#include "xslam/opencv/utils.h"

#include "gtest/gtest.h"
#include "glog/logging.h"

namespace xslam {
namespace opencv {

TEST(PoseEstimation2D2D, pose_estimation_2d)
{
    std::string image1 = GetOpenCVDatasetDirectory() + "/0003_orb_feature_detector.png";
    std::string image2 = GetOpenCVDatasetDirectory() + "/0004_orb_feature_detector.png";

    // OpenCV
    PoseEstimation2D2D demo;
    demo.RunDemo(image1, image2);
}

} // namespace opencv
} // namespace xslam