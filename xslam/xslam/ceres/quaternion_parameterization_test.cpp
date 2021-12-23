//
// Created by quan on 2021/12/20.
//

#include "xslam/ceres/quaternion_parameterization.h"
#include "glog/logging.h"
#include "gtest/gtest.h"

namespace xslam {
namespace ceres {

TEST(QuaternionParameter, Parameter)
{
    LOG(INFO) << "Start RobustCurveFitting .... ";
    QuaternionParameter demo;
    demo.RunDemo();
}

}
}