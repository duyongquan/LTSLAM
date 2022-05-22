#ifndef XSLAM_OPENCV_VIDEO_ANALYSIS_CAM_SHIFT_H
#define XSLAM_OPENCV_VIDEO_ANALYSIS_CAM_SHIFT_H

#include "glog/logging.h"
#include "opencv2/opencv.hpp"

#include <string>

namespace xslam {
namespace opencv {
namespace video_analysis {
    
class Camshift
{
public:
    void RunDemo(const std::string& filename);
};

} // namespace video_analysis
} // namespace opencv
} // namespace xslam

#endif // XSLAM_OPENCV_VIDEO_ANALYSIS_CAM_SHIFT_H