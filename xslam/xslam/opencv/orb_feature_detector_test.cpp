//
// Created by quan on 2021/12/14.
//

#include "xslam/opencv/orb_feature_detector.h"

#include "xslam/opencv/corner_harris.h"
#include "xslam/opencv/utils.h"

#include "gtest/gtest.h"
#include "glog/logging.h"

namespace xslam {
namespace opencv {

TEST(ORBFeatureDetector, OpenCVcornerDetect)
{
    std::string image1 = GetOpenCVDatasetDirectory() + "/0003_orb_feature_detector.png";
    std::string image2 = GetOpenCVDatasetDirectory() + "/0004_orb_feature_detector.png";

    // OpenCV
    ORBFeatureDetector demo;
    demo.CornerDetect(image1, image2);
}

TEST(ORBFeatureDetector, SelfcornerDetect)
{
    std::string image1 = GetOpenCVDatasetDirectory() + "/0003_orb_feature_detector.png";
    std::string image2 = GetOpenCVDatasetDirectory() + "/0004_orb_feature_detector.png";

    // Myself
    ORBFeatureDetector demo;
    demo.SelfCornerDetect(image1, image2);
}

} // namespace opencv
} // namespace xslam