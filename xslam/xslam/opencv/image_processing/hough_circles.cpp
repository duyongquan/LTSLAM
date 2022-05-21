#include "xslam/opencv/image_processing/hough_circles.h"

#include <vector>

namespace xslam {
namespace opencv {
namespace image_processing {

void HoughCircles::RunDemo(const std::string& filename)
{
    // 0. 读取图像
    cv::Mat image = cv::imread(filename);
    if (image.data == nullptr) {
        std::cout << "Load image error." << std::endl;
        exit(-1);
    }

	cv::imshow("原图", image);
    cv::Mat mout;
    cv::medianBlur(image, mout, 7);            // 中值滤波降噪
    cv::cvtColor(mout, mout, cv::COLOR_BGR2GRAY);   // 转换为灰度图像
    std::vector<cv::Vec3f> circles;                 // 存储圆的容器
    cv::HoughCircles(mout, circles, cv::HOUGH_GRADIENT, 1, 10, 100, 30, 5, 100); // 进行霍夫圆检测
    cv::Scalar circleColor = cv::Scalar(255, 0, 0);   // 圆形的边缘颜色
    cv::Scalar centerColor = cv::Scalar(0, 0, 255); // 圆心的颜色

    for (int i = 0; i < circles.size(); i++) {
        cv::Vec3f c = circles[i];
        cv::circle(image, cv::Point(c[0], c[1]),c[2], circleColor, 2, cv::LINE_AA);  // 画边缘
        cv::circle(image, cv::Point(c[0], c[1]), 2, centerColor, 2, cv::LINE_AA);    // 画圆心
    }
    cv::imshow("dst", image); // 显示处理后的图像

    while (true) {
        if (27 == cv::waitKey()) { // ESC key
            break;
        } 
        sleep(1);
    }
    cv::destroyAllWindows();
}

} // namespace image_processing
} // namespace opencv
} // namespace xslam
