//
// Created by quan on 2021/12/9.
//

#include "xslam/ceres/simple_bundle_adjuster.h"
#include "glog/logging.h"
#include "gtest/gtest.h"

namespace xslam {
namespace ceres {

TEST(SimpleBundleAdjuster, RejectionError)
{
    LOG(INFO) << "Start SimpleBundleAdjuster .... ";
    SimpleBundleAdjuster demo;
    demo.RunDemo();
}

} // namespace ceres
} // namespace xslam
