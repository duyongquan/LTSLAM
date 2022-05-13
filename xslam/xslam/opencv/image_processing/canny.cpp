#include "xslam/opencv/image_processing/canny.h"

namespace xslam {
namespace opencv {
namespace image_processing {

void Canny::RunDemo(const std::string& filename)
{
    // , grayImage;
    cv::Mat image = cv::imread(filename);
    if (image.data == nullptr) {
        std::cout << "Load image error." << std::endl;
        exit(-1);
    }

    cv::Mat grayImage;
    cv::Mat srcImage1 = image.clone();
    cv::cvtColor(image, grayImage, cv::COLOR_BGR2GRAY);
    cv::Mat dstImage, edge;
 
    cv::blur(grayImage, grayImage, cv::Size(3,3));
    cv::Canny(grayImage, edge, 150, 100, 3);
 
    dstImage.create(srcImage1.size(), srcImage1.type());
    srcImage1.copyTo(dstImage, edge);

    cv::imshow("origin", image);
    cv::imshow("canny", dstImage);
    cv::waitKey(0);
    cv::destroyAllWindows();
}

} // namespace image_processing
} // namespace opencv
} // namespace xslam
