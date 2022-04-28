#include "xslam/g2o/pcg.h"

#include "gtest/gtest.h"
#include "glog/logging.h"

namespace xslam {
namespace g2o {

TEST(PCGSolver, demo)
{
    std::cout << "Run PCGSolver solve Ax = b.\n";
    PCGSolver pcg;
    pcg.RunDemo();
}

} // namespace g2o
} // namespace xslam