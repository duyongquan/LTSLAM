#include "xslam/g2o/icp.h"

#include "gtest/gtest.h"
#include "glog/logging.h"

namespace xslam {
namespace g2o {

TEST(ICP, demo_point)
{
    LOG(INFO) << "Run G2O ICP kPoint2Point.";
    
    ICP demo;
    demo.RunDemo(ICP::Mode::kPoint2Point);
}

TEST(ICP, demo_plane)
{
    LOG(INFO) << "Run G2O ICP kPoint2Plane.";
    
    ICP demo;
    demo.RunDemo(ICP::Mode::kPoint2Plane);
}


} // namespace g2o
} // namespace xslam
