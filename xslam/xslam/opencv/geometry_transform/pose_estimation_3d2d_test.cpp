#include "xslam/opencv/geometry_transform/pose_estimation_3d2d.h"
#include "xslam/opencv/utils.h"

#include "gtest/gtest.h"
#include "glog/logging.h"

namespace xslam {
namespace opencv {
namespace geometry_transform {

TEST(PoseEstimation3D2D, pose_estimation_2d)
{
    std::string image1 = GetOpenCVDatasetDirectory() + "/0024_1.png";
    std::string depth1 = GetOpenCVDatasetDirectory() + "/0024_1_depth.png";
    std::string image2 = GetOpenCVDatasetDirectory() + "/0024_2.png";
    std::string depth2 = GetOpenCVDatasetDirectory() + "/0024_2_depth.png";

    // OpenCV
    PoseEstimation3D2D demo;
    demo.RunDemo(image1, depth1, image2, depth2);
}

} // namespace geometry_transform
} // namespace opencv
} // namespace xslam


