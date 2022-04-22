#include "xslam/g2o/descent_gradient_method.h"

#include "xslam/opencv/utils.h"

#include "gtest/gtest.h"
#include "glog/logging.h"

namespace xslam {
namespace g2o {
    
TEST(DescentGradient, demo)
{
    LOG(INFO) << "Run G2O Steepest-Descent-Gradient-Method.";
    DescentGradient demo;
    demo.RunDemo();
}

} // namespace g2o
} // namespace xslam