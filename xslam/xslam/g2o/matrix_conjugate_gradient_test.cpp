#include "xslam/g2o/matrix_conjugate_gradient.h"

#include "gtest/gtest.h"
#include "glog/logging.h"

namespace xslam {
namespace g2o {

TEST(MatrixConjugateGradientSolver, demo)
{
    MatrixConjugateGradientSolver demo;
    demo.RunDemo();
}

} // namespace g2o
} // namespace xslam