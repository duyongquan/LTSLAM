//
// Created by quan on 2021/12/7.
//

#include "xslam/ceres/curve_fitting.h"
#include "glog/logging.h"
#include "gtest/gtest.h"

namespace xslam {
namespace ceres {

TEST(CurveFitting, fitting)
{
    LOG(INFO) << "Start CurveFitting .... ";
    CurveFitting demo;
    demo.RunDemo();
}

}
}