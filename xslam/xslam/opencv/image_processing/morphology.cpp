#include "xslam/opencv/image_processing/morphology.h"

namespace xslam {
namespace opencv {
namespace image_processing {

void Morphology::RunDemo(const std::string& filename)
{
    // 0. 读取图像
    cv::Mat image = cv::imread(filename);
    if (image.data == nullptr) {
        std::cout << "Load image error." << std::endl;
        exit(-1);
    }

    // erode
    cv::Mat erode;
	//自定义核
	cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(8, 8));
	cv::imshow("原图", image);
	cv::erode(image, erode, element);
	cv::imshow("腐蚀", erode);

    // dilate
	cv::Mat dilate_element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
	cv::Mat dilate;
	cv::dilate(image, dilate, dilate_element);
	imshow("dilate", dilate);

    // morphologyEx
    cv::Mat morphologyEx;
    //定义核
    cv::Mat morphologyEx_element(7, 7, CV_8U, cv::Scalar(1));
    //进行形态学开运算操作
    cv::morphologyEx(image, morphologyEx, cv::MORPH_OPEN, morphologyEx_element);
    imshow("形态学开运算", morphologyEx);

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
