#include "xslam/opencv/image_processing/calcHist.h"

#include <vector>

namespace xslam {
namespace opencv {
namespace image_processing {

void CalcHist::RunDemo(const std::string& filename)
{
    // 0. 读取图像
    cv::Mat image = cv::imread(filename);
    if (image.data == nullptr) {
        std::cout << "Load image error." << std::endl;
        exit(-1);
    }

	cv::imshow("原图", image);

    //分割成三通道图像
    std::vector<cv::Mat> channels;
    cv::split(image, channels);

    //设定bin数目
    int histBinNum = 255;

    //设定取值范围
    float range[] = {0, 255};
    const float* histRange = {range};

    bool uniform = true;
    bool accumulate = false;

    //声明三个通道的hist数组
    cv::Mat red_hist, green_hist, blue_hist;

    //计算直方图
    cv::calcHist(&channels[0], 1, 0, cv::Mat(), red_hist, 1, &histBinNum, &histRange, uniform, accumulate);
    cv::calcHist(&channels[1], 1, 0, cv::Mat(), green_hist, 1, &histBinNum, &histRange, uniform, accumulate);
    cv::calcHist(&channels[2], 1, 0, cv::Mat(), blue_hist, 1, &histBinNum, &histRange, uniform, accumulate);

    //创建直方图窗口
    int hist_w = 400;
    int hist_h = 400;
    int bin_w = cvRound((double)image.cols/histBinNum);

    cv::Mat histImage(image.cols, image.rows, CV_8UC3, cv::Scalar(0, 0, 0));

    //将直方图归一化到范围[0, histImage.rows]
    cv::normalize(red_hist, red_hist, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat());
    cv::normalize(green_hist, green_hist, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat());
    cv::normalize(blue_hist, blue_hist, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat());

    //循环绘制直方图
    for(int i = 1; i < histBinNum; i++)
    {
        cv::line(histImage, cv::Point(bin_w*(i-1), image.rows - cvRound(red_hist.at<float>(i-1))),
            cv::Point(bin_w*(i), image.rows - cvRound(red_hist.at<float>(i))), cv::Scalar(0, 0, 255), 2, 8, 0);
        cv::line(histImage, cv::Point(bin_w*(i-1), image.rows - cvRound(green_hist.at<float>(i-1))),
            cv::Point(bin_w*(i), image.rows - cvRound(green_hist.at<float>(i))), cv::Scalar(0, 255, 0), 2, 8, 0);
        cv::line(histImage, cv::Point(bin_w*(i-1), image.rows - cvRound(blue_hist.at<float>(i-1))),
            cv::Point(bin_w*(i), image.rows - cvRound(blue_hist.at<float>(i))), cv::Scalar(255, 0, 0), 2, 8, 0);
    }

    cv::namedWindow("原图像", cv::WINDOW_AUTOSIZE);
    cv::imshow("原图像", image);

    cv::namedWindow("图像直方图", cv::WINDOW_AUTOSIZE);
    cv::imshow("图像直方图", histImage);

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
