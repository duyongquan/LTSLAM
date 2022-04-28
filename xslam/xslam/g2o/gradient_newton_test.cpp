#include "xslam/g2o/gradient_newton.h"

#include "gtest/gtest.h"
#include "glog/logging.h"

namespace xslam {
namespace g2o {

TEST(GradientNewtonSolver, demo)
{
    GradientNewtonSolver demo;
    demo.RunDemo();
}

} // namespace g2o
} // namespace xslam