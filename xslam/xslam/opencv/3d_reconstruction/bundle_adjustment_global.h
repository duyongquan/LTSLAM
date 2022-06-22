#ifndef XSLAM_OPENCV_3D_RECONSTRUCTION_BUNDLE_ADJUSTMENT_GLOBAL_H
#define XSLAM_OPENCV_3D_RECONSTRUCTION_BUNDLE_ADJUSTMENT_GLOBAL_H

#include "xslam/opencv/geometry_transform/bundle_adjustment.h"

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

class BundleAdjustmentGlobal
{
public:
    void RunDemo(const std::string& filename);
};

} // namespace reconstruction
} // namespace opencv
} // namespace xslam

#endif // XSLAM_OPENCV_3D_RECONSTRUCTION_BUNDLE_ADJUSTMENT_GLOBAL_H
