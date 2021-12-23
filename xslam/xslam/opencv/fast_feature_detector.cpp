//
// Created by quan on 2021/12/14.
//

#include "xslam/opencv/fast_feature_detector.h"

#include "opencv2/xfeatures2d.hpp"

namespace xslam {
namespace opencv {

void FastFeature::CornerDetect(const std::string& filename)
{
    // 1 read a image
    cv::Mat image = cv::imread(filename);
    if (image.data == nullptr) {
        std::cout << "Load image error." << std::endl;
        exit(-1);
    }

    // 2 convert to gray
    cv::Mat gray;
    cv::cvtColor(image, gray,cv::COLOR_BGR2GRAY);

    std::vector<cv::KeyPoint> keypoints;
    cv::Mat dst = image.clone();
    cv::Ptr<cv::FastFeatureDetector> detector = cv::FastFeatureDetector::create(40);
    detector->detect(image,keypoints);
    drawKeypoints(dst, keypoints, dst, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_OVER_OUTIMG);

    cv::imshow("FastFeature Corner Detected", dst);
    cv::waitKey(0);
    cv::destroyAllWindows();
}

} // namespace opencv
} // namespace xslam
