#ifndef XSLAM_OPENCV_IMAGE_PROCESSING_GEOMETRY_TRANSFORM_H
#define XSLAM_OPENCV_IMAGE_PROCESSING_GEOMETRY_TRANSFORM_H

#include "glog/logging.h"
#include "opencv2/opencv.hpp"

#include <string>

namespace xslam {
namespace opencv {
namespace image_processing {
    
class GeometryTransform
{
public:
    void RunDemo(const std::string& filename);

private:
    // resize
    void Resize(const cv::Mat& image);

    // getAffineTransform
    void AffineTransform(const cv::Mat& image);

    // warpPerspective
    void WarpPerspective(const cv::Mat& image);

};

} // namespace image_processing
} // namespace opencv
} // namespace xslam

#endif // XSLAM_OPENCV_IMAGE_PROCESSING_GEOMETRY_TRANSFORM_H