//
// Created by quan on 2021/12/15.
//

#ifndef SLAM_KLT_H
#define SLAM_KLT_H


#include "opencv2/opencv.hpp"
#include "glog/logging.h"

namespace xslam {
namespace opencv {

class OpticalFlowPyrLK
{
public:
    void Run(const std::string& filename);

private:
    void Tracking(cv::Mat &frame, cv::Mat &output);
    bool AddNewPoints();
    bool AcceptTrackedPoint(int i);

    cv::Mat gray;
    cv::Mat gray_prev;

    std::vector<cv::Point2f> points[2];
    std::vector<cv::Point2f> initial;
    std::vector<cv::Point2f> features;

    int maxCount = 500;
    double qLevel = 0.01;
    double minDist = 10.0;
    std::vector<uchar> status;
    std::vector<float> err;
};

} // namespace opencv
} // namespace xslam


#endif //SLAM_KLT_H
