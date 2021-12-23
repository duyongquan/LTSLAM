//
// Created by quan on 2021/12/14.
//

#include "xslam/opencv/corner_harris.h"

namespace xslam {
namespace opencv {

void CornerHarris::CornerDetect(const std::string& filename)
{
    // 1 read a image
    cv::Mat image = cv::imread(filename);
    if (image.data == nullptr) {
        std::cout << "Load image error." << std::endl;
        exit(-1);
    }

    // 2 convert to gray
    cv::Mat gray;
    cv::cvtColor(image, gray,cv::COLOR_BGR2GRAY);

    // 3 cornerHarris角点检测
    // 进行角点检测
    // 领域大小为 2
    // sobel 算子孔径 3
    // harris 参数
    cv::Mat dstImage;       //目标图
    cv::Mat normImage;      //归一化后的图
    cv::Mat scaledImage;    //线性变换后的八位无符号整型的图

    //置零当前需要显示的两幅图，即清除上一次调用此函数时他们的值
    dstImage = cv::Mat::zeros(image.size(), CV_32FC1 );
    cv::cornerHarris(gray, dstImage, 2, 3, 0.04, cv::BORDER_DEFAULT );

    // 归一化与转换
    cv::normalize( dstImage, normImage, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat() );
    convertScaleAbs( normImage, scaledImage );      //将归一化后的图线性变换成8位无符号整型

    // 4、进行绘制
    // 将检测到的，且符合阈值条件的角点绘制出来
    int corner_count = 0;
    for( int j = 0; j < normImage.rows ; j++ )
        for( int i = 0; i < normImage.cols; i++ )
        {
            if( (int) normImage.at<float>(j,i) > 80 )    //  设定阈值
            {
                cv::circle(image, cv::Point( i, j ), 6,  cv::Scalar(0,255,5), 2, 1, 0 );
            }
        }

    // 5 显示最终效果
    cv::imshow("CornerHarris Corner Detected", image);
    cv::waitKey(0);
    cv::destroyAllWindows();
}

} // namespace opencv
} // namespace xslam

