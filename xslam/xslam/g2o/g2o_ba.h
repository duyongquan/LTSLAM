#ifndef XSLAM_G2O_G2O_BA_H
#define XSLAM_G2O_G2O_BA_H

#include <vector>
#include <string>
#include <iostream>

// for opencv
#include <boost/concept_check.hpp>
#include "opencv2/opencv.hpp"

// for g2o
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/types/slam3d/se3quat.h>
#include <g2o/types/sba/types_six_dof_expmap.h>


// 在这个程序中，我们读取两张图像，进行特征匹配。然后根据匹配得到的特征，计算相机运动以及特征点的位置。
// 这是一个典型的Bundle Adjustment，我们用g2o进行优化。

namespace xslam {
namespace g2o {

class SimpleBA
{
public:
    void RunDemo(const std::string& image1, const std::string& image2);

private:
    // 寻找两个图像中的对应点，像素坐标系
    // 输入：img1, img2 两张图像
    // 输出：points1, points2, 两组对应的2D点
    int FindCorrespondingPoints(const cv::Mat& img1, const cv::Mat& img2, 
        std::vector<cv::Point2f>& points1, std::vector<cv::Point2f>& points2);

    // 相机内参
    const double cx = 325.5;
    const double cy = 253.5;
    const double fx = 518.0;
    const double fy = 519.0;
};

} // namespace g2o
} // namespace xslam


#endif // XSLAM_G2O_G2O_BA_H