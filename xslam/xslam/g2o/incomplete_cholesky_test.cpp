#include "xslam/g2o/incomplete_cholesky.h"

#include "gtest/gtest.h"
#include "glog/logging.h"

namespace xslam {
namespace g2o {

TEST(IncompleteCholeskySolver, demo)
{
    IncompleteCholeskySolver demo;
    demo.RunDemo();
}

} // namespace g2o
} // namespace xslam