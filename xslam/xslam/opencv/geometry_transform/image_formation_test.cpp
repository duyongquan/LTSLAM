#include "xslam/opencv/geometry_transform/image_formation.h"
#include "xslam/opencv/utils.h"

#include "gtest/gtest.h"
#include "glog/logging.h"

namespace xslam {
namespace opencv {
namespace geometry_transform {

TEST(ImageFormation, image_formation)
{
    std::string image = GetOpenCVDatasetDirectory() + "/box.xyz";
    ImageFormation demo;
    demo.RunDemo(image);
}

} // namespace geometry_transform
} // namespace opencv
} // namespace xslam