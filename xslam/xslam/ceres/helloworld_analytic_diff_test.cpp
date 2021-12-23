//
// Created by quan on 2021/12/7.
//

#include "xslam/ceres/helloworld_analytic_diff.h"
#include "glog/logging.h"
#include "gtest/gtest.h"

namespace xslam {
namespace ceres {

TEST(HelloWorldAnalyticDiff, LinearTest)
{
    LOG(INFO) << "Start AnalyticDiff .... ";
    AnalyticDiff demo;
    demo.RunDemo();
}

}
}

