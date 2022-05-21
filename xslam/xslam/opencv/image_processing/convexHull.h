#ifndef XSLAM_OPENCV_IMAGE_PROCESSING_CONVEXHULL_H
#define XSLAM_OPENCV_IMAGE_PROCESSING_CONVEXHULL_H

#include "glog/logging.h"
#include "opencv2/opencv.hpp"

#include <string>

namespace xslam {
namespace opencv {
namespace image_processing {

void ThreshCallback(int, void*);

class ConvexHull
{
public:
    void RunDemo(const std::string& filename);
};

} // namespace image_processing
} // namespace opencv
} // namespace xslam

#endif // XSLAM_OPENCV_IMAGE_PROCESSING_CONVEXHULL_H