//
// Created by quan on 2021/12/20.
//

#include "xslam/g2o/curve_fitting.h"

#include "gtest/gtest.h"
#include "glog/logging.h"

namespace xslam {
namespace g2o {

TEST(CurveFitting, demo)
{
    CurveFitting demo;
    demo.RunDemo();
}

} // namespace g2o
} // namespace xslam