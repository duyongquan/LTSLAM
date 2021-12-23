
//
// Created by quan on 2021/12/20.
//

#ifndef SLAM_RGBD_MAP_H
#define SLAM_RGBD_MAP_H

#include "opencv2/opencv.hpp"
#include "glog/logging.h"

#include <sophus/se3.hpp>
#include <Eigen/Core>
#include <vector>
#include <string>

namespace xslam {
namespace opencv {

class RGBDMap
{
public:
    using TrajectoryType = std::vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>>;
    using Vector6d = Eigen::Matrix<double, 6, 1>;

    void RunDemo(const std::string& filename,
                 const std::string& color_and_depth_filename_dir);

private:
    void ShowPointCloud(const std::vector<Vector6d, Eigen::aligned_allocator<Vector6d>> &pointcloud);
};

} // namespace opencv
} // namespace xslam


#endif //SLAM_RGBD_MAP_H
