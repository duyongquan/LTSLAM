//
// Created by quan on 2021/12/14.
//

#ifndef SLAM_ORB_FEATURE_DETECTOR_H
#define SLAM_ORB_FEATURE_DETECTOR_H

#include "opencv2/opencv.hpp"
#include "glog/logging.h"

#include <vector>

namespace xslam {
namespace opencv {

class ORBFeatureDetector
{
public:
    // OpenCV implement
    void CornerDetect(const std::string& image1, const std::string& image2);

    // Self implement
    void SelfCornerDetect(const std::string& image1, const std::string& image2);

private:
    // 32 bit unsigned int, will have 8, 8x32=256
    typedef std::vector<uint32_t> DescType; // Descriptor type

    /**
     * compute descriptor of orb keypoints
     * @param img input image
     * @param keypoints detected fast keypoints
     * @param descriptors descriptors
     *
     * NOTE: if a keypoint goes outside the image boundary (8 pixels), descriptors will not be computed and will be left as
     * empty
     */
    void ComputeORB(const cv::Mat &img,
            std::vector<cv::KeyPoint> &keypoints,
            std::vector<DescType> &descriptors);

    /**
     * brute-force match two sets of descriptors
     * @param desc1 the first descriptor
     * @param desc2 the second descriptor
     * @param matches matches of two images
     */
    void BfMatch(const std::vector<DescType> &desc1,
                 const std::vector<DescType> &desc2,
                 std::vector<cv::DMatch> &matches);

};

} // namespace opencv
} // namespace xslam


#endif //SLAM_ORB_FEATURE_DETECTOR_H
