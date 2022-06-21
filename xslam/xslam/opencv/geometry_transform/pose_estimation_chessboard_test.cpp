#include "xslam/opencv/geometry_transform/pose_estimation_chessboard.h"
#include "xslam/opencv/utils.h"

#include "gtest/gtest.h"
#include "glog/logging.h"

namespace xslam {
namespace opencv {
namespace geometry_transform {

TEST(PoseEstimationChessboard, pose_estimation_chessboard)
{
    std::string avi = GetOpenCVDatasetDirectory() + "/chessboard.avi";
    PoseEstimationChessboard demo;
    demo.RunDemo(avi);
}

} // namespace geometry_transform
} // namespace opencv
} // namespace xslam