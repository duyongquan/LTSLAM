#include "xslam/opencv/image_basic/clahe.h"

namespace xslam {
namespace opencv {
namespace image_basic {

void CLAHE::RunDemo(const std::string& filename)
{
    cv::Mat image = cv::imread(filename, 0);
    if (image.data == nullptr) { //数据不存在,可能是文件不存在
        std::cerr << "文件: " << filename << "不存在." << std::endl;
        return;
    }

    cv::Mat clahe;
    cv::Ptr<cv::CLAHE> clahe_ptr = cv::createCLAHE();
    clahe_ptr->apply(image, clahe);

    cv::imshow("source image",image);
    cv::imshow("clahe image",clahe);
    cv::waitKey();
}

} // namespace image_basic
} // namespace opencv
} // namespace xslam
