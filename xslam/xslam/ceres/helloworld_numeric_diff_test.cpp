//
// Created by quan on 2021/12/7.
//

#include "xslam/ceres/helloworld_numeric_diff.h"
#include "glog/logging.h"
#include "gtest/gtest.h"

namespace xslam {
namespace ceres {

TEST(HelloWorldNumericDiff, LinearTest)
{
    LOG(INFO) << "Start NumericDiff .... ";
    NumericDiff demo;
    demo.RunDemo();
}

}
}