#include "xslam/opencv/geometry_transform/object_localization.h"
#include "xslam/opencv/utils.h"

#include "gtest/gtest.h"
#include "glog/logging.h"

namespace xslam {
namespace opencv {
namespace geometry_transform {

TEST(ObjectLocalization, pose_estimation_chessboard)
{
    std::string image = GetOpenCVDatasetDirectory() + "/0026_daejeon_station.png";
    ObjectLocalization demo;
    demo.RunDemo(image);
}

} // namespace geometry_transform
} // namespace opencv
} // namespace xslam