//
// Created by quan on 2021/12/14.
//

#ifndef SLAM_FAST_FEATURE_DETECTOR_H
#define SLAM_FAST_FEATURE_DETECTOR_H

#include "opencv2/opencv.hpp"
#include "glog/logging.h"

namespace xslam {
namespace opencv {

class FastFeature
{
public:
    void CornerDetect(const std::string& filename);
};

} // namespace opencv
} // namespace xslam

#endif //SLAM_FAST_FEATURE_DETECTOR_H
