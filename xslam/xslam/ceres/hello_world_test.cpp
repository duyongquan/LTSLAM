//
// Created by quan on 2021/12/6.
//

#include "xslam/ceres/hello_world.h"
#include "glog/logging.h"
#include "gtest/gtest.h"


namespace xslam {
namespace ceres {

TEST(HelloWorld, LinearTest)
{
    LOG(INFO) << "Start .... ";
    HelloWorld demo;
    demo.RunDemo();
}

}
}