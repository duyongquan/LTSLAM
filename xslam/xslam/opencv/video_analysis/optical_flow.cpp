#include "xslam/opencv/video_analysis/optical_flow.h"

#include <vector>

namespace xslam {
namespace opencv {
namespace video_analysis {

void OpticalFlow::RunDemo(const std::string& filename)
{
	cv::Mat image1, image2;
	std::vector<cv::Point2f> point1, point2, pointCopy;
	std::vector<uchar> status;
	std::vector<float> err;

    cv::VideoCapture video(filename);
	video >> image1;
	cv::Mat image1Gray, image2Gray;
	cv::cvtColor(image1, image1Gray, cv::COLOR_RGB2GRAY);
	cv::goodFeaturesToTrack(image1Gray, point1, 100, 0.01, 10, cv::Mat());
	pointCopy = point1;

	for (int i = 0; i < point1.size(); i++)    //绘制特征点位  
	{
		cv::circle(image1, point1[i], 1, cv::Scalar(0, 0, 255), 2);
	}

	cv::namedWindow("光流特征图");
	while (true)
	{
		video >> image2;
		if (cv::waitKey(33) == ' ')  //按下空格选择当前画面作为标定图像  
		{
			cv::cvtColor(image2, image1Gray, cv::COLOR_RGB2GRAY);
			cv::goodFeaturesToTrack(image1Gray, point1, 100, 0.01, 10, cv::Mat());
			pointCopy = point1;
		}
		cv::cvtColor(image2, image2Gray, cv::COLOR_RGB2GRAY);
		cv::calcOpticalFlowPyrLK(image1Gray, image2Gray, point1, point2, status, err, cv::Size(50, 50), 3); // LK金字塔       
		int tr_num = 0;
		auto status_itr = status.begin();
		while (status_itr != status.end()) {
			if (*status_itr > 0)
				tr_num++;
			status_itr++;
		}
		if (tr_num < 6) {
			std::cout << "you need to change the feat-img because the background-img was all changed" << std::endl;
			if (cv::waitKey(0) == ' ') {
				cv::cvtColor(image2, image1Gray, cv::COLOR_RGB2GRAY);
				cv::goodFeaturesToTrack(image1Gray, point1, 100, 0.01, 10, cv::Mat());
				pointCopy = point1;
			}
		}
		for (int i = 0; i < point2.size(); i++)
		{
			cv::circle(image2, point2[i], 1, cv::Scalar(0, 0, 255), 2);
			cv::line(image2, pointCopy[i], point2[i], cv::Scalar(255, 0, 0), 2);
		}

		cv::imshow("光流特征图", image2);
		std::swap(point1, point2);
		image1Gray = image2Gray.clone();
	}
}

} // namespace video_analysis
} // namespace opencv
} // namespace xslam
