#ifndef XSLAM_OPENCV_IMAGE_STITCHING_H
#define SLAM_SHI_TXSLAM_OPENCV_IMAGE_STITCHING_HOMASI_H

#include "opencv2/opencv.hpp"
#include "glog/logging.h"

namespace xslam {
namespace opencv {
namespace feature_detection {

class ImageStitching
{
public:
    void RunDemo(const std::string& image1, const std::string& image2);
};

} // namespace feature_detection
} // namespace opencv
} // namespace xslam


#endif //XSLAM_OPENCV_IMAGE_STITCHING_H
