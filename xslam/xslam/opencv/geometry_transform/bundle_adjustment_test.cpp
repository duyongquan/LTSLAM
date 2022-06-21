#include "xslam/opencv/geometry_transform/bundle_adjustment.h"

#include "xslam/opencv/utils.h"

#include "gtest/gtest.h"
#include "glog/logging.h"

namespace xslam {
namespace opencv {
namespace geometry_transform {


TEST(BundleAdjustment, bundle_adjustment_demo)
{
    std::string images = GetOpenCVDatasetDirectory() + "/image_formation%d.xyz";
    BundleAdjustment demo;
    demo.RunDemo(images);
}


} // namespace geometry_transform
} // namespace opencv
} // namespace xslam