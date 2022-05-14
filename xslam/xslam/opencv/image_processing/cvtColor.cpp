#include "xslam/opencv/image_processing/cvtColor.h"

namespace xslam {
namespace opencv {
namespace image_processing {

void CvtColor::RunDemo(const std::string& filename)
{
    // 0. 读取图像
    cv::Mat image = cv::imread(filename);
    if (image.data == nullptr) {
        std::cout << "Load image error." << std::endl;
        exit(-1);
    }
    // 原图
    cv::imshow("image", image); // 展示源图像

    // 灰度图
    cv::Mat img1, img2, img3;
    cv::cvtColor(image, img1, cv::COLOR_RGB2GRAY);
    cv::imshow("灰度图", img1);

    // HSV
    cv::cvtColor(image, img2, cv::COLOR_RGB2HSV);
	cv::imshow("HSV", img2);

    // BGR
    cv::cvtColor(image, img3, cv::COLOR_RGB2BGR);
	cv::imshow("BGR", img3);
  
    cv::waitKey(0);
    cv::destroyAllWindows();
}

} // namespace image_processing
} // namespace opencv
} // namespace xslam
