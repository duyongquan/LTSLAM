#include "xslam/opencv/image_processing/pyr_down_up.h"

namespace xslam {
namespace opencv {
namespace image_processing {

void PyrDownUp::RunDemo(const std::string& filename)
{
    // 0. 读取图像
    cv::Mat image = cv::imread(filename);
    if (image.data == nullptr) {
        std::cout << "Load image error." << std::endl;
        exit(-1);
    }

	cv::imshow("原图", image);
	
	cv::Mat out;
	cv::pyrDown(image, out);
	cv::imshow("降采样", out);
	cv::pyrUp(out, out);
	cv::imshow("上采样", out);
	cv::subtract(image, out, out);
	cv::imshow("拉普拉斯金字塔", out);

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
