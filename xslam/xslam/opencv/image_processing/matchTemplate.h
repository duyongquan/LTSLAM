#ifndef XSLAM_OPENCV_IMAGE_PROCESSING_MATCH_TEMPLATE_H
#define XSLAM_OPENCV_IMAGE_PROCESSING_MATCH_TEMPLATE_H

#include "glog/logging.h"
#include "opencv2/opencv.hpp"

#include <string>

namespace xslam {
namespace opencv {
namespace image_processing {
    
class MatchTemplate
{
public:
    void RunDemo(const std::string& filename, const std::string& template_file);
};

} // namespace image_processing
} // namespace opencv
} // namespace xslam

#endif // XSLAM_OPENCV_IMAGE_PROCESSING_MATCH_TEMPLATE_H