#ifndef XSLAM_OPENCV_3D_RECONSTRUCTION_SFM_INC_H
#define XSLAM_OPENCV_3D_RECONSTRUCTION_SFM_INC_H

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

class SFMInc
{
public:
    void RunDemo(const std::string& filename);

private:
    cv::Mat getCameraMat(const SFM::Vec9d& camera);
    cv::Mat getProjectionMat(const SFM::Vec9d& camera);
    void updateCameraPose(SFM::Vec9d& camera, const cv::Mat& R, const cv::Mat& t);

    bool isBadPoint(
        const cv::Point3d& X, const SFM::Vec9d& camera1, 
        const SFM::Vec9d& camera2, 
        double Z_limit, double max_cos_parallax);
};

} // namespace reconstruction
} // namespace opencv
} // namespace xslam

#endif // XSLAM_OPENCV_3D_RECONSTRUCTION_SFM_INC_H
