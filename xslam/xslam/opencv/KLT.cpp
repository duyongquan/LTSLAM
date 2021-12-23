//
// Created by quan on 2021/12/15.
//

#include "xslam/opencv/KLT.h"

namespace xslam {
namespace opencv {

void OpticalFlowPyrLK::Run(const std::string& filename)
{
    cv::Mat frame;
    cv::Mat result;
    cv::VideoCapture capture(filename);

    if(capture.isOpened())
    {
        while(true) {
            capture >> frame;
            if(!frame.empty()) {
                Tracking(frame, result);
            }
            else {
                LOG(INFO) << " --(!) No captured frame -- Break!";
                break;
            }

            int c = cv::waitKey(50);
            if( (char)c == 27 ) {
                break;
            }
        }
    }
}

void OpticalFlowPyrLK::Tracking(cv::Mat &frame, cv::Mat &output)
{

    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    frame.copyTo(output);

    if (AddNewPoints())
    {
        cv::goodFeaturesToTrack(gray, features, maxCount, qLevel, minDist);
        points[0].insert(points[0].end(), features.begin(), features.end());
        initial.insert(initial.end(), features.begin(), features.end());
    }

    if (gray_prev.empty()) {
        gray.copyTo(gray_prev);
    }

    cv::calcOpticalFlowPyrLK(gray_prev, gray, points[0], points[1], status, err);

    int k = 0;
    for (int i = 0; i < points[1].size(); i++)
    {
        if (AcceptTrackedPoint(i)) {
            initial[k] = initial[i];
            points[1][k++] = points[1][i];
        }
    }
    points[1].resize(k);
    initial.resize(k);

    for (int i = 0; i < points[1].size(); i++) {
        cv::line(output, initial[i], points[1][i], cv::Scalar(0, 0, 255));
        cv::circle(output, points[1][i], 3, cv::Scalar(0, 255, 0), -1);
    }

    std::swap(points[1], points[0]);
    std::swap(gray_prev, gray);

    cv::imshow("KLT OpticalFlowPyrLK", output);
}


bool OpticalFlowPyrLK::AddNewPoints()
{
    return points[0].size() <= 10;
}

bool OpticalFlowPyrLK::OpticalFlowPyrLK::AcceptTrackedPoint(int i)
{
    return status[i] && ((std::abs(points[0][i].x - points[1][i].x) + abs(points[0][i].y - points[1][i].y)) > 2);
}

} // namespace opencv
} // namespace xslam
