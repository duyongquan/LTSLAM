#include "xslam/opencv/image_processing/image_gradient.h"

namespace xslam {
namespace opencv {
namespace image_processing {

void Gradient::RunDemo(const std::string& filename)
{
    // 0. 读取图像
    cv::Mat image = cv::imread(filename);
    if (image.data == nullptr) {
        std::cout << "Load image error." << std::endl;
        exit(-1);
    }

    cv::namedWindow("image", cv::WINDOW_AUTOSIZE);
    cv::imshow("image", image); 

    // 1. 高斯模糊
    cv::Mat srcBlur;
    cv::GaussianBlur(image, srcBlur, cv::Size(3, 3), 0, 0);

    // 2. 转灰度
    cv::Mat srcGray;
    cvtColor(srcBlur, srcGray, cv::COLOR_BGR2GRAY);

    // 3. 求方向梯度
    cv::Mat gradX, gradY;
    cv::Sobel(srcGray, gradX, CV_16S, 1, 0, 3);
    cv::Sobel(srcGray, gradY, CV_16S, 0, 1, 3);

    cv::convertScaleAbs(gradX, gradX);  // calculates absolute values, and converts the result to 8-bit.
    cv::convertScaleAbs(gradY, gradY);
    cv::namedWindow("gradY", cv::WINDOW_AUTOSIZE);
    cv::imshow("gradX", gradX);
    cv::namedWindow("gradY", cv::WINDOW_AUTOSIZE);
    cv::imshow("gradY", gradY);

    printf("type: %d, %d", gradX.type(), gradY.type());

    // 4. 图像混合
    cv::Mat dst;
    cv::addWeighted(gradX, 0.5, gradY, 0.5, 0, dst);
    cv::namedWindow("dst", cv::WINDOW_AUTOSIZE);
    cv::imshow("dst", dst);

    // 4.1 
    cv::Mat gradXY = cv::Mat(gradX.size(), gradX.type());
    for (int row = 0; row < gradX.rows; row++) 
    {
        for (int col = 0; col < gradX.cols; col++)
        {
            int gX = gradX.at<uchar>(row, col);
            int gY = gradY.at<uchar>(row, col);
            gradXY.at<uchar>(row, col) = cv::saturate_cast<uchar>(gX + gY);
        }
    }
    cv::namedWindow("gradXY", cv::WINDOW_AUTOSIZE);
    cv::imshow("gradXY", gradXY);
    cv::waitKey(0);
    cv::destroyAllWindows();
}

} // namespace image_processing
} // namespace opencv
} // namespace xslam
