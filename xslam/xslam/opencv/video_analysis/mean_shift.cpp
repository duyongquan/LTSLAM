#include "xslam/opencv/video_analysis/mean_shift.h"

#include <vector>

using namespace cv;
using namespace std;

namespace xslam {
namespace opencv {
namespace video_analysis {

void Meanshift::RunDemo(const std::string& filename)
{

    VideoCapture capture(filename);
    if (!capture.isOpened()){
        //error in opening the video input
        cerr << "Unable to open file!" << endl;
    }

    Mat frame, roi, hsv_roi, mask;
    // take first frame of the video
    capture >> frame;

    // setup initial location of window
    Rect track_window(300, 200, 100, 50); // simply hardcoded the values

    // set up the ROI for tracking
    roi = frame(track_window);
    cvtColor(roi, hsv_roi, COLOR_BGR2HSV);
    inRange(hsv_roi, Scalar(0, 60, 32), Scalar(180, 255, 255), mask);

    float range_[] = {0, 180};
    const float* range[] = {range_};
    Mat roi_hist;
    int histSize[] = {180};
    int channels[] = {0};
    calcHist(&hsv_roi, 1, channels, mask, roi_hist, 1, histSize, range);
    normalize(roi_hist, roi_hist, 0, 255, NORM_MINMAX);

    // Setup the termination criteria, either 10 iteration or move by atleast 1 pt
    TermCriteria term_crit(TermCriteria::EPS | TermCriteria::COUNT, 10, 1);

    while(true){
        Mat hsv, dst;
        capture >> frame;
        if (frame.empty())
            break;
        cvtColor(frame, hsv, COLOR_BGR2HSV);
        calcBackProject(&hsv, 1, channels, roi_hist, dst, range);

        // apply meanshift to get the new location
        meanShift(dst, track_window, term_crit);

        // Draw it on image
        rectangle(frame, track_window, 255, 2);
        imshow("img2", frame);

        int keyboard = waitKey(30);
        if (keyboard == 'q' || keyboard == 27)
            break;
    }
}

} // namespace video_analysis
} // namespace opencv
} // namespace xslam
