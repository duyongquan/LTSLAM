//
// Created by quan on 2021/12/20.
//

#ifndef SLAM_TRIANGULATION_H
#define SLAM_TRIANGULATION_H

#include <opencv2/opencv.hpp>
#include <vector>

namespace xslam {
namespace opencv {

class Triangulation
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
            const std::vector<cv::KeyPoint> &keypoints_1,
            const std::vector<cv::KeyPoint> &keypoints_2,
            const std::vector<cv::DMatch>   &matches,
            cv::Mat &R, cv::Mat &t);

    void DoTriangulation(
            const std::vector<cv::KeyPoint> &keypoint_1,
            const std::vector<cv::KeyPoint> &keypoint_2,
            const std::vector<cv::DMatch>   &matches,
            const cv::Mat &R, const cv::Mat &t,
            std::vector<cv::Point3d> &points);

    cv::Scalar GetColor(float depth);

    // 像素坐标转相机归一化坐标
    cv::Point2f Pixel2Cam(const cv::Point2d &p, const cv::Mat &K);
};

} // namespace opencv
} // namespace xslam

#endif //SLAM_TRIANGULATION_H
