#include "xslam/opencv/image_pyramid.h"

namespace xslam {
namespace opencv {

void ImagePyramid::PyrDown(const std::string& filename, int level)
{
    cv::Mat image = cv::imread(filename);

    if (image.data == nullptr) {
        std::cout << "Load image error." << std::endl;
        exit(-1);
    }

    cv::Mat pyramid = image.clone();

    for (int i = level; i > 0; i--)
    {
        cv::imshow("PyrDown" + std::to_string(i), pyramid);
        cv::pyrDown(image, pyramid, cv::Size(image.cols * i / level, image.rows * i / level)); 
    }
    
    cv::waitKey(0);
    cv::destroyAllWindows();
}

void ImagePyramid::PyrUp(const std::string& filename, int level)
{
    
}

} // namespace opencv
} // namespace xslam