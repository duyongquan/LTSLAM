#include "xslam/opencv/image_processing/hough_lines.h"

#include <vector>

namespace xslam {
namespace opencv {
namespace image_processing {

void HoughLines::RunDemo(const std::string& filename)
{
    // 0. 读取图像
    cv::Mat image = cv::imread(filename);
    if (image.data == nullptr) {
        std::cout << "Load image error." << std::endl;
        exit(-1);
    }

	cv::imshow("原图", image);
	cv::Mat mid, dst;
	cv::Canny(image, mid, 100, 200, 3);
	cv::cvtColor(mid, dst, cv::COLOR_GRAY2BGR);

	// 标准霍夫变换，直线检测
	std::vector<cv::Vec2f> lines;
	cv::HoughLines(mid, lines, 1, CV_PI / 180.0, 200, 0, 0);
	for (size_t i = 0; i < lines.size(); ++i)
	{
		float rho = lines[i][0], theta = lines[i][1];
		cv::Point pt1, pt2;
		double a = cos(theta), b = sin(theta);
		double x0 = a * rho, y0 = b * rho;
		pt1.x = cvRound(x0 + 1000 * (-b));
		pt1.y = cvRound(y0 + 1000 * (a));
		pt2.x = cvRound(x0 - 1000 * (-b));
		pt2.y = cvRound(y0 - 1000 * (a));
		cv::line(dst, pt1, pt2, cv::Scalar(255, 0, 0), 1, cv::LINE_AA);
	}

	cv::imshow("mid", mid);
	cv::imshow("result", dst);
    while (true) {
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
