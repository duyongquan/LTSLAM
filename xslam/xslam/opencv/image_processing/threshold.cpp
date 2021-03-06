#include "xslam/opencv/image_processing/threshold.h"

namespace xslam {
namespace opencv {
namespace image_processing {

void Threshold::RunDemo(const std::string& filename)
{
    // 0. 读取图像
    cv::Mat image = cv::imread(filename, cv::IMREAD_GRAYSCALE);
    if (image.data == nullptr) {
        std::cout << "Load image error." << std::endl;
        exit(-1);
    }

	cv::imshow("原图", image);
	cv::Mat dst;
 
	double thresh = 100;
	int maxVal = 255;
	cv::threshold(image, dst, thresh, maxVal, cv::THRESH_BINARY);
	cv::imshow("threshold", dst);

    while (true) 
    {
        if (27 == cv::waitKey()) {
            break;
        } 

        sleep(1);
    }
    
    cv::destroyAllWindows();
}

} // namespace image_processing
} // namespace opencv
} // namespace xslam
