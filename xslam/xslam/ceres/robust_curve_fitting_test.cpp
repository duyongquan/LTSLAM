//
// Created by quan on 2021/12/7.
//

#include "xslam/ceres/robust_curve_fitting.h"
#include "glog/logging.h"
#include "gtest/gtest.h"

namespace xslam {
namespace ceres {

TEST(RobustCurveFitting, fitting)
{
    LOG(INFO) << "Start RobustCurveFitting .... ";
    RobustCurveFitting demo;
    demo.RunDemo();
}

}
}