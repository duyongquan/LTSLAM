#include "xslam/g2o/levenberg_marquardt.h"

#include "gtest/gtest.h"
#include "glog/logging.h"

namespace xslam {
namespace g2o {

TEST(LevenbergMarquardt, demo)
{
    LOG(INFO) << "Run gaussian new function solve Ax = b.";
    LevenbergMarquardt demo;
    demo.RunDemo();
}

} // namespace g2o
} // namespace xslam