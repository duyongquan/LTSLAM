#include "xslam/opencv/3d_reconstruction/bundle_adjustment_global.h"
#include "xslam/opencv/utils.h"

#include "gtest/gtest.h"
#include "glog/logging.h"

namespace xslam {
namespace opencv {
namespace reconstruction {

TEST(BundleAdjustmentGlobal, bundle_adjustment_global)
{
    std::string avi = GetOpenCVDatasetDirectory() + "/image_formation%d.xyz";
    BundleAdjustmentGlobal demo;
    demo.RunDemo(avi);
}

} // namespace reconstruction
} // namespace opencv
} // namespace xslam
