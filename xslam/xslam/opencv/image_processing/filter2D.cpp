#include "xslam/opencv/image_processing/filter2D.h"

namespace xslam {
namespace opencv {
namespace image_processing {

void Filter2D::RunDemo(const std::string& filename)
{
    // 0. 读取图像
    cv::Mat image = cv::imread(filename);
    if (image.data == nullptr) {
        std::cout << "Load image error." << std::endl;
        exit(-1);
    }

    cv::imshow("image", image); // 展示源图像
    cv::Mat dst;
    cv::Mat kernal = (cv::Mat_<char>(3, 3) << 0, -1, 0, -1, 5, -1, 0, -1, 0); // 生成卷积核
    cv::filter2D(image, dst, -1, kernal);
    cv::imshow("dst", dst); // 展示目标图像
    cv::waitKey(0);
    cv::destroyAllWindows();
}

} // namespace image_processing
} // namespace opencv
} // namespace xslam
