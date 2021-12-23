//
// Created by quan on 2021/12/20.
//

#include "xslam/ceres/size_cost_function.h"
#include "glog/logging.h"
#include "gtest/gtest.h"

namespace xslam {
namespace ceres {

TEST(RobustCurveFitting, demo01)
{
    LOG(INFO) << "Start RobustCurveFitting .... ";
    SizeCostFunction demo;
    demo.RunDemo();
}

}
}