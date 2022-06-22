#include "xslam/opencv/geometry_transform/pose_estimation.h"
#include "xslam/opencv/utils.h"

#include "gtest/gtest.h"
#include "glog/logging.h"

namespace xslam {
namespace opencv {
namespace geometry_transform {

TEST(PoseEstimation, demo01)
{
    // const char *input = "data/blais.mp4", *cover = "data/blais.jpg";
    std::string vedio = GetOpenCVDatasetDirectory() + "/blais.mp4";
    std::string image = GetOpenCVDatasetDirectory() + "/blais.jpg";
    PoseEstimation demo;
    demo.RunDemo1(vedio, image);
}

TEST(PoseEstimation, demo02)
{
    std::string vedio = GetOpenCVDatasetDirectory() + "/blais.mp4";
    std::string image = GetOpenCVDatasetDirectory() + "/blais.jpg";
    PoseEstimation demo;
    demo.RunDemo2(vedio, image);
}

TEST(PoseEstimation, demo03)
{
    std::string vedio = GetOpenCVDatasetDirectory() + "/blais.mp4";
    std::string image = GetOpenCVDatasetDirectory() + "/blais.jpg";
    PoseEstimation demo;
    demo.RunDemo3(vedio, image);
}

} // namespace geometry_transform
} // namespace opencv
} // namespace xslam