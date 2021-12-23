//
// Created by quan on 2021/12/14.
//

#include "xslam/opencv/shi_tomasi.h"

namespace xslam {
namespace opencv {

void ShiTomasi::CornerDetect(const std::string& filename)
{
    cv::Mat image = cv::imread(filename);
    if (image.data == nullptr) {
        std::cout << "Load image error." << filename << std::endl;
        exit(-1);
    }

    cv::Mat gray;
    cv::cvtColor(image, gray,cv::COLOR_BGR2GRAY);

    std::vector<cv::Point2f> corners;
    goodFeaturesToTrack(gray, corners, 100, 0.01,50, cv::Mat());
    for(int i = 0; i < corners.size(); i++) {
        // image，背景图
        // center，圆心
        // radius，半径
        // color，颜色
        // thickness，线粗细
        circle(image, corners[i],5,cv::Scalar(0,0,255),2);
    }

    imshow("Shi-Tomasi Corner Detected",image);
    cv::waitKey(0);
    cv::destroyAllWindows();
}

} // namespace opencv
} // namespace xslam