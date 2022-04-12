//
// Created by quan on 2021/12/20.
//

#ifndef SLAM_STEREO_VISION_H
#define SLAM_STEREO_VISION_H

#include "opencv2/opencv.hpp"
#include "glog/logging.h"

#include <Eigen/Core>
#include <vector>
#include <string>

namespace xslam {
namespace opencv {

class StereoVision
{
public:
    void RunDemo(const std::string& image_left, const std::string& image_right);

private:
    void ShowPointCloud(const std::vector<Eigen::Vector4d,
                        Eigen::aligned_allocator<Eigen::Vector4d>> &pointcloud);
};

} // namespace opencv
} // namespace slam

#endif //SLAM_STEREO_VISION_H
