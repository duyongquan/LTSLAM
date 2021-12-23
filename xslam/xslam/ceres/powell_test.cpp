//
// Created by quan on 2021/12/7.
//

#include "xslam/ceres/powell.h"

#include "glog/logging.h"
#include "gtest/gtest.h"

namespace xslam {
namespace ceres {

TEST(Powell, MultiFunction)
{
    LOG(INFO) << "Start Powell .... ";
    Powell demo;
    demo.RunDemo();
}

}
}