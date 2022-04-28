#include "xslam/g2o/damped_newton.h"

#include "gtest/gtest.h"
#include "glog/logging.h"

namespace xslam {
namespace g2o {

TEST(DampedNewtonSolver, demo)
{
    DampedNewtonSolver demo;
    demo.RunDemo();
}

} // namespace g2o
} // namespace xslam