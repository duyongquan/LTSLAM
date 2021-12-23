//
// Created by quan on 2021/12/20.
//

#include "xslam/opencv/triangulation.h"

namespace xslam {
namespace opencv {

void Triangulation::RunDemo(const std::string& iamge_1, const std::string& iamge_2)
{
    //-- 读取图像
    cv::Mat img_1 = cv::imread(iamge_1);
    cv::Mat img_2 = cv::imread(iamge_2);

    std::vector<cv::KeyPoint> keypoints_1, keypoints_2;
    std::vector<cv::DMatch> matches;
    FindFeatureMatches(img_1, img_2, keypoints_1, keypoints_2, matches);
    std::cout << "一共找到了" << matches.size() << "组匹配点" << std::endl;

    //-- 估计两张图像间运动
    cv::Mat R, t;
    PoseEstimation_2D2D(keypoints_1, keypoints_2, matches, R, t);

    //-- 三角化
    std::vector<cv::Point3d> points;
    DoTriangulation(keypoints_1, keypoints_2, matches, R, t, points);

    //-- 验证三角化点与特征点的重投影关系
    cv::Mat K = (cv::Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
    cv::Mat img1_plot = img_1.clone();
    cv::Mat img2_plot = img_2.clone();
    for (int i = 0; i < matches.size(); i++)
    {
        // 第一个图
        float depth1 = points[i].z;
        std::cout << "depth: " << depth1 << std::endl;
        cv::Point2d pt1_cam = Pixel2Cam(keypoints_1[matches[i].queryIdx].pt, K);
        cv::circle(img1_plot, keypoints_1[matches[i].queryIdx].pt, 2, GetColor(depth1), 2);

        // 第二个图
        cv::Mat pt2_trans = R * (cv::Mat_<double>(3, 1) << points[i].x, points[i].y, points[i].z) + t;
        float depth2 = pt2_trans.at<double>(2, 0);
        cv::circle(img2_plot, keypoints_2[matches[i].trainIdx].pt, 2, GetColor(depth2), 2);
    }
    cv::imshow("img 1", img1_plot);
    cv::imshow("img 2", img2_plot);
    cv::waitKey();
}

void Triangulation::FindFeatureMatches(
        const cv::Mat &img_1, const cv::Mat &img_2,
        std::vector<cv::KeyPoint> &keypoints_1,
        std::vector<cv::KeyPoint> &keypoints_2,
        std::vector<cv::DMatch> &matches)
{
    //-- 初始化
    cv::Mat descriptors_1, descriptors_2;

    // used in OpenCV3
    cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
    cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();

    // use this if you are in OpenCV2
    // Ptr<FeatureDetector> detector = FeatureDetector::create ( "ORB" );
    // Ptr<DescriptorExtractor> descriptor = DescriptorExtractor::create ( "ORB" );
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
    //-- 第一步:检测 Oriented FAST 角点位置
    detector->detect(img_1, keypoints_1);
    detector->detect(img_2, keypoints_2);

    //-- 第二步:根据角点位置计算 BRIEF 描述子
    descriptor->compute(img_1, keypoints_1, descriptors_1);
    descriptor->compute(img_2, keypoints_2, descriptors_2);

    //-- 第三步:对两幅图像中的BRIEF描述子进行匹配，使用 Hamming 距离
    std::vector<cv::DMatch> match;
    // BFMatcher matcher ( NORM_HAMMING );
    matcher->match(descriptors_1, descriptors_2, match);

    //-- 第四步:匹配点对筛选
    double min_dist = 10000, max_dist = 0;

    //找出所有匹配之间的最小距离和最大距离, 即是最相似的和最不相似的两组点之间的距离
    for (int i = 0; i < descriptors_1.rows; i++) {
        double dist = match[i].distance;
        if (dist < min_dist) min_dist = dist;
        if (dist > max_dist) max_dist = dist;
    }

    printf("-- Max dist : %f \n", max_dist);
    printf("-- Min dist : %f \n", min_dist);

    //当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
    for (int i = 0; i < descriptors_1.rows; i++) {
        if (match[i].distance <= std::max(2 * min_dist, 30.0)) {
            matches.push_back(match[i]);
        }
    }
}

void Triangulation::PoseEstimation_2D2D(
        const std::vector<cv::KeyPoint> &keypoints_1,
        const std::vector<cv::KeyPoint> &keypoints_2,
        const std::vector<cv::DMatch>   &matches,
        cv::Mat &R, cv::Mat &t)
{
    // 相机内参,TUM Freiburg2
    cv::Mat K = (cv::Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);

    //-- 把匹配点转换为vector<Point2f>的形式
    std::vector<cv::Point2f> points1;
    std::vector<cv::Point2f> points2;

    for (int i = 0; i < (int) matches.size(); i++)
    {
        points1.push_back(keypoints_1[matches[i].queryIdx].pt);
        points2.push_back(keypoints_2[matches[i].trainIdx].pt);
    }

    //-- 计算本质矩阵
    cv::Point2d principal_point(325.1, 249.7);        //相机主点, TUM dataset标定值
    int focal_length = 521;            //相机焦距, TUM dataset标定值
    cv::Mat essential_matrix;
    essential_matrix = cv::findEssentialMat(points1, points2, focal_length, principal_point);

    //-- 从本质矩阵中恢复旋转和平移信息.
    cv::recoverPose(essential_matrix, points1, points2, R, t, focal_length, principal_point);
}

void Triangulation::DoTriangulation(
        const std::vector<cv::KeyPoint> &keypoint_1,
        const std::vector<cv::KeyPoint> &keypoint_2,
        const std::vector<cv::DMatch>   &matches,
        const cv::Mat &R, const cv::Mat &t,
        std::vector<cv::Point3d> &points)
{
    cv::Mat T1 = (cv::Mat_<float>(3, 4) <<
            1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0);

    cv::Mat T2 = (cv::Mat_<float>(3, 4) <<
            R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), t.at<double>(0, 0),
            R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), t.at<double>(1, 0),
            R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), t.at<double>(2, 0)
    );

    cv::Mat K = (cv::Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
    std::vector<cv::Point2f> pts_1, pts_2;

    for (cv::DMatch m : matches) {
        // 将像素坐标转换至相机坐标
        pts_1.push_back(Pixel2Cam(keypoint_1[m.queryIdx].pt, K));
        pts_2.push_back(Pixel2Cam(keypoint_2[m.trainIdx].pt, K));
    }

    cv::Mat pts_4d;
    cv::triangulatePoints(T1, T2, pts_1, pts_2, pts_4d);

    // 转换成非齐次坐标
    for (int i = 0; i < pts_4d.cols; i++) {
        cv::Mat x = pts_4d.col(i);
        x /= x.at<float>(3, 0); // 归一化
        cv::Point3d p(
                x.at<float>(0, 0),
                x.at<float>(1, 0),
                x.at<float>(2, 0)
        );
        points.push_back(p);
    }
}


cv::Scalar Triangulation::GetColor(float depth)
{
    float up_th = 50, low_th = 10, th_range = up_th - low_th;
    if (depth > up_th) {
        depth = up_th;
    }
    if (depth < low_th) {
        depth = low_th;
    }
    return cv::Scalar(255 * depth / th_range, 0, 255 * (1 - depth / th_range));
}

cv::Point2f Triangulation::Pixel2Cam(const cv::Point2d &p, const cv::Mat &K)
{
    return cv::Point2f
    {
        (p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
        (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1)
    };
}

} // namespace opencv
} // namespace xslam