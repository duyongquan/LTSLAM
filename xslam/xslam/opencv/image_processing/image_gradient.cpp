#include "xslam/opencv/image_processing/image_gradient.h"

namespace xslam {
namespace opencv {
namespace image_processing {

void Gradient::RunDemo(const std::string& filename)
{
    // , grayImage;
    cv::Mat image = cv::imread(filename);
    if (image.data == nullptr) {
        std::cout << "Load image error." << std::endl;
        exit(-1);
    }

   
    // cv::imshow("canny", dstImage);
    cv::waitKey(0);
    cv::destroyAllWindows();
}

} // namespace image_processing
} // namespace opencv
} // namespace xslam
