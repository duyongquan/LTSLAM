#include "xslam/g2o/icp.h"

#include "gtest/gtest.h"
#include "glog/logging.h"

namespace xslam {
namespace g2o {

TEST(ICP, demo_point)
{
    LOG(INFO) << "Run G2O ICP.";
    
    ICP demo;
    demo.RunDemo();
}

} // namespace g2o
} // namespace xslam
