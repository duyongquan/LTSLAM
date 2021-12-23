//
// Created by quan on 2021/12/14.
//

#include "xslam/opencv/read_image.h"

namespace xslam {
namespace opencv {

void ReadImage::Read(const std::string& filename)
{
    cv::Mat image = cv::imread(filename);

    if (image.data == nullptr) {
        std::cout << "Load image error." << std::endl;
        exit(-1);
    }

    cv::imshow("Opencv read image", image);
    cv::waitKey(0);
    cv::destroyAllWindows();
}

} // namespace opencv
} // namespace xslam