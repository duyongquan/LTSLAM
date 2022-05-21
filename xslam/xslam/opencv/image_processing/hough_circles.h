#ifndef XSLAM_OPENCV_IMAGE_PROCESSING_HOUGH_CIRCLES_H
#define XSLAM_OPENCV_IMAGE_PROCESSING_HOUGH_CIRCLES_H

#include "glog/logging.h"
#include "opencv2/opencv.hpp"

#include <string>

namespace xslam {
namespace opencv {
namespace image_processing {
    
class HoughCircles
{
public:
    void RunDemo(const std::string& filename);
};

} // namespace image_processing
} // namespace opencv
} // namespace xslam

#endif // XSLAM_OPENCV_IMAGE_PROCESSING_HOUGH_CIRCLES_H