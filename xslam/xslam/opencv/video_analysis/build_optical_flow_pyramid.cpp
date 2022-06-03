#include "xslam/opencv/video_analysis/build_optical_flow_pyramid.h"

#include <vector>

using namespace std;

namespace xslam {
namespace opencv {
namespace video_analysis {

void BuildOpticalFlowPyramid::RunDemo(const std::string& filename)
{
    cv::VideoCapture cap(filename);
    if(!cap.isOpened()){
        std::cerr << "cannot open camera\n";
    }

    cv::Mat frame,gray,grayPre,framePre,status,err;
    const int maxLevel = 3;
    std::vector<cv::Point2f> prevPts, nextPts;
    std::vector<cv::Mat> pyramid1,pyramid2;

    cap >> frame;
    if(frame.empty()){
        std::cerr << "grab first frame error.\n";
    }

    cv::cvtColor(frame,grayPre,cv::COLOR_BGR2GRAY);
    cv::Size subPixWinSize(10,10);
    cv::TermCriteria termcrit(cv::TermCriteria::COUNT|cv::TermCriteria::EPS,20,0.03);
    while(cap.isOpened())
    {
        cap >> frame;
        if(frame.empty()){
            std::cerr << "cannot grab frame from camera.\n";
            break;
        }

        cv::cvtColor(frame,gray,cv::COLOR_BGR2GRAY);

        // 检测触点
        goodFeaturesToTrack(gray, nextPts, 100, 0.01, 2.0);
        cornerSubPix(gray, nextPts, subPixWinSize, cv::Size(-1,-1), termcrit);
        goodFeaturesToTrack(grayPre, prevPts, 100, 0.01, 2.0);
        cornerSubPix(gray, prevPts, subPixWinSize, cv::Size(-1,-1), termcrit);

        // 构造流光金字塔
        cv::buildOpticalFlowPyramid(gray, pyramid1, cv::Size(21,21), maxLevel);
        cv::buildOpticalFlowPyramid(grayPre, pyramid2, cv::Size(21,21), maxLevel);

        // 使用LK流光算法检测
        cv::calcOpticalFlowPyrLK(pyramid1,pyramid2,prevPts,nextPts,status,err);
        gray.copyTo(grayPre);

        size_t i, k;
        for( i = k = 0; i < nextPts.size(); i++ ){
            cv::circle( frame, nextPts[i], 3, cv::Scalar(0,0,255), -1, 8);
        }

        // 显示图像
        cv::imshow("frame",frame);
        if(cv::waitKey(10) == 27){
            break;
        }
    }
}

} // namespace video_analysis
} // namespace opencv
} // namespace xslam
