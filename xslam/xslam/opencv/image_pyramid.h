#ifndef XSLAM_OPENCV_IMAGE_PYRAMID_H
#define XSLAM_OPENCV_IMAGE_PYRAMID_H

#include "opencv2/opencv.hpp"
#include "glog/logging.h"

#include <string>

namespace xslam {
namespace opencv {

class ImagePyramid
{
public:
    void PyrDown(const std::string& filename, int level);
    void PyrUp(const std::string& filename, int level);
};

} // namespace opencv
} // namespace xslam

#endif // XSLAM_OPENCV_IMAGE_PYRAMID_H