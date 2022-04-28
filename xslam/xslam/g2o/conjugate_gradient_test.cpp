#include "xslam/g2o/conjugate_gradient.h"

#include "gtest/gtest.h"
#include "glog/logging.h"

namespace xslam {
namespace g2o {

TEST(ConjugateGradientSolver, demo)
{
    ConjugateGradientSolver demo;
    demo.RunDemo();
}

} // namespace g2o
} // namespace xslam