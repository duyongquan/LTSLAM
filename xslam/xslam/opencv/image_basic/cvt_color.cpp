#include "xslam/opencv/image_basic/cvt_color.h"

namespace xslam {
namespace opencv {
namespace image_basic {

void CvtColor::RunDemo(const std::string& filename)
{
    cv::Mat image = cv::imread(filename);
    if (image.data == nullptr) { //数据不存在,可能是文件不存在
        std::cerr << "文件: " << filename << "不存在." << std::endl;
        return;
    }

    //显示原图像
    cv::namedWindow("原图像", cv::WINDOW_AUTOSIZE);
    cv::imshow("原图像",image);

    //将图像转换为灰度图，采用CV_前缀
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);     //将图像转换为灰度图
    cv::namedWindow("灰度图", cv::WINDOW_AUTOSIZE);
    cv::imshow("灰度图",gray);

    //将图像转换为HSV，采用COLOR_前缀
    cv::Mat hsv;
    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);    //将图像转换为HSV图
    cv::namedWindow("HSV", cv::WINDOW_AUTOSIZE);
    cv::imshow("HSV",hsv);
    cv::waitKey(0);
}

} // namespace image_basic
} // namespace opencv
} // namespace xslam
