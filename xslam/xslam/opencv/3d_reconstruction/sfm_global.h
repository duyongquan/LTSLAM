#ifndef XSLAM_OPENCV_3D_RECONSTRUCTION_SFM_GLOBAL_H
#define XSLAM_OPENCV_3D_RECONSTRUCTION_SFM_GLOBAL_H

#include "xslam/opencv/3d_reconstruction/sfm.hpp"

#include "opencv2/opencv.hpp"
#include "glog/logging.h"
#include "ceres/ceres.h"
#include "ceres/rotation.h"

#include <sophus/se3.hpp>
#include <Eigen/Core>
#include <vector>
#include <string>
#include <unordered_map>


namespace xslam {
namespace opencv {
namespace reconstruction {

class SFMGlobal
{
public:
    void RunDemo(const std::string& filename);

private:
    std::vector<bool> maskNoisyPoints(
        std::vector<cv::Point3d>& Xs, 
        const std::vector<std::vector<cv::KeyPoint>>& xs, 
        const std::vector<SFM::Vec9d>& views, 
        const SFM::VisibilityGraph& visibility, 
        double reproj_error2);
};

} // namespace reconstruction
} // namespace opencv
} // namespace xslam

#endif // XSLAM_OPENCV_3D_RECONSTRUCTION_SFM_GLOBAL_H
