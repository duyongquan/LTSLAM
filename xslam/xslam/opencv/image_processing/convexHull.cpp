#include "xslam/opencv/image_processing/convexHull.h"

#include <vector>
#include <functional>

namespace xslam {
namespace opencv {
namespace image_processing {

cv::RNG rng(12345);
cv::Mat src_gray;
int thresh = 100;

void ConvexHull::RunDemo(const std::string& filename)
{
    // 0. 读取图像
    cv::Mat image = cv::imread(filename, 0);
    if (image.data == nullptr) {
        std::cout << "Load image error." << std::endl;
        exit(-1);
    }

	cv::imshow("原图", image);
    
    cvtColor( image, src_gray, cv::COLOR_BGR2GRAY );
    cv::blur( src_gray, src_gray, cv::Size(3,3) );
    const char* source_window = "Source";

    const int max_thresh = 255;
    // cv::createTrackbar( "Canny thresh:", source_window, &thresh, max_thresh, 
    //     std::bind(&ConvexHull::ThreshCallback, std::placeholders::_1, std::placeholders::_2));

    cv::createTrackbar( "Canny thresh:", source_window, &thresh, max_thresh, ThreshCallback);

    ThreshCallback( 0, 0 );

    while (true) {
        if (27 == cv::waitKey()) {
            break;
        } 
        sleep(1);
    }
    cv::destroyAllWindows();
}

void ThreshCallback(int, void*)
{
    cv::Mat canny_output;
    cv::Canny( src_gray, canny_output, thresh, thresh*2 );
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours( canny_output, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE );
    std::vector<std::vector<cv::Point> >hull( contours.size() );
    for( size_t i = 0; i < contours.size(); i++ )
    {
        cv::convexHull( contours[i], hull[i] );
    }
    cv::Mat drawing =cv:: Mat::zeros( canny_output.size(), CV_8UC3 );
    for( size_t i = 0; i< contours.size(); i++ )
    {
        cv::Scalar color = cv::Scalar( rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256) );
        cv::drawContours( drawing, contours, (int)i, color );
        cv::drawContours( drawing, hull, (int)i, color );
    }
    cv::imshow( "Hull demo", drawing );
}


} // namespace image_processing
} // namespace opencv
} // namespace xslam
