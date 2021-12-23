//
// Created by quan on 2021/12/20.
//

#ifndef SLAM_POSE_ESTIMATION_2D2D_H
#define SLAM_POSE_ESTIMATION_2D2D_H

#include "opencv2/opencv.hpp"
#include "glog/logging.h"

#include <string>

namespace xslam {
namespace opencv {

/**
 * 本程序演示了如何使用2D-2D的特征匹配估计相机运动
 */
class PoseEstimation2D2D
{
public:
    void RunDemo(const std::string& iamge_1, const std::string& iamge_2);

private:
    void FindFeatureMatches(
            const cv::Mat &img_1, const cv::Mat &img_2,
            std::vector<cv::KeyPoint> &keypoints_1,
            std::vector<cv::KeyPoint> &keypoints_2,
            std::vector<cv::DMatch> &matches);

    void PoseEstimation_2D2D(
            std::vector<cv::KeyPoint> keypoints_1,
            std::vector<cv::KeyPoint> keypoints_2,
            std::vector<cv::DMatch> matches,
            cv::Mat &R, cv::Mat &t);

    // 像素坐标转相机归一化坐标
    cv::Point2d Pixel2Cam(const cv::Point2d &p, const cv::Mat &K);
};

} // namespace opencv
} // namespace xslam

#endif //SLAM_POSE_ESTIMATION_2D2D_H
