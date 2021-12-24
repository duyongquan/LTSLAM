
#include "xslam/opencv/paint_shape.h"


namespace xslam {
namespace opencv {

void PaintShape::RunDemo()
{
    // 1 line
    PaintLine();

    // 2 circle
    PaintCircle();

    // Sets the
    PaintRectangle();

    // 4 rectangle
    PaintRectangle();
}

void PaintShape::PaintLine()
{
    // 创建黑色的图像
    cv::Mat image = cv::Mat(512, 512, CV_8UC3);

    // 绘制一条厚度为5的蓝色对角线
    cv::line(image, cv::Point(0, 0), cv::Point(511, 511), cv::Scalar(255, 0, 0), 5);

    cv::imshow("PaintShape", image);
    cv::waitKey(0);
    cv::destroyAllWindows();
}   

void PaintShape::PaintCircle()
{
    // 创建黑色的图像
    cv::Mat image = cv::Mat(512, 512, CV_8UC3);

    // 绘制一条厚度为5的蓝色对角线
    cv::circle(image, cv::Point(447, 63), 63, cv::Scalar(0, 0, 255), -1);

    cv::imshow("PaintShape", image);
    cv::waitKey(0);
    cv::destroyAllWindows();
}

void PaintShape::PaintRectangle()
{
    // 创建黑色的图像
    cv::Mat image = cv::Mat(512, 512, CV_8UC3);

    // 绘制一条厚度为5的蓝色对角线
    cv::rectangle(image, cv::Point(384, 0), cv::Point(510, 128), cv::Scalar(0, 255, 0), 3);

    cv::imshow("PaintShape", image);
    cv::waitKey(0);
    cv::destroyAllWindows();
}


} // namespace opencv
} // namespace xslam