#include "xslam/g2o/gauss_newton.h"

#include "gtest/gtest.h"
#include "glog/logging.h"

namespace xslam {
namespace g2o {

TEST(GuassianNewtonMethod, demo)
{
    LOG(INFO) << "Run gaussian new function solve Ax = b.";
    GuassianNewtonMethod demo;
    demo.RunDemo();
}

} // namespace g2o
} // namespace xslam