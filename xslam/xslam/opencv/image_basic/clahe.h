#ifndef XSLAM_OPENCV_IMAGE_BASIC_CLAHE_H
#define XSLAM_OPENCV_IMAGE_BASIC_CLAHE_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <boost/dynamic_bitset.hpp>

namespace xslam {
namespace opencv {
namespace image_basic {

// CLAHE（Contrast Limited Adaptive Histogram Equalization)
// 灰度图中像素值的分布为0-255，以灰度值为横坐标，纵坐标为该灰度值对应的像素点数目/比例，则得到了灰度图像的直方图，
// 体现的是图像中灰度的整体分布情况。
// OpenCV中提供了相应的计算函数calcHist()，可以得到相应分布范围的像素点数；
// 直方图均衡化就是一种使图像的灰度直方图分布更佳均匀的变换方法
// cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();

class CLAHE
{
public:
    void RunDemo(const std::string& filename);
};

} // namespace image_basic
} // namespace opencv
} // namespace xslam

#endif // XSLAM_OPENCV_IMAGE_BASIC_CLAHE_H