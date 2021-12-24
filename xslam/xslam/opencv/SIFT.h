#ifndef XSLAM_OPENCV_SIFT_H
#define XSLAM_OPENCV_SIFT_H

#include "opencv2/opencv.hpp"
#include "glog/logging.h"
#include "vlfeat/sift.h"

#include <string>

namespace xslam {
namespace opencv {
    
class SIFTFeature
{
public:
    void RunDemo(const std::string& filename);

private:
    void ExtractFeature(VlSiftFilt* sift_ptr, vl_sift_pix* data, 
        std::vector<VlSiftKeypoint>& keypoints, 
        std::vector<float*>& descriptors);

    void ConvertToOpencvKeypoint(std::vector<VlSiftKeypoint>& vlfeat_keypoints,
                                 std::vector<cv::KeyPoint>& opencv_keypoints);

    VlSiftFilt* vl_sift_;
};

} // namespace opencv
} // namespace xslam

#endif // XSLAM_OPENCV_SIFT_H