#include "xslam/opencv/geometry_transform/vo_epipolar.h"
#include "xslam/opencv/utils.h"

#include "gtest/gtest.h"
#include "glog/logging.h"


namespace xslam {
namespace opencv {
namespace geometry_transform {

TEST(VOEpipolar, vo_epipolar)
{
    std::string filename = GetOpenCVDatasetDirectory() + "/07/image_0/%06d.png";
    VOEpipolar demo;
    demo.RunDemo(filename);
}

} // namespace geometry_transform
} // namespace opencv
} // namespace xslam