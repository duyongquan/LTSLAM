#include "xslam/opencv/geometry_transform/triangulation_1.h"
#include "xslam/opencv/utils.h"

#include "gtest/gtest.h"
#include "glog/logging.h"


namespace xslam {
namespace opencv {
namespace geometry_transform {

TEST(Triangulation, triangulation)
{
    std::string image1 = GetOpenCVDatasetDirectory() + "/image_formation0.xyz";
    std::string image2 = GetOpenCVDatasetDirectory() + "/image_formation1.xyz";

    Triangulation demo;
    demo.RunDemo(image1, image2);
}

} // namespace geometry_transform
} // namespace opencv
} // namespace xslam