#include "xslam/g2o/levenberg_marquardt.h"

#include "gtest/gtest.h"
#include "glog/logging.h"

namespace xslam {
namespace g2o {

// https://zhuanlan.zhihu.com/p/42415718
TEST(LMCurveFitting, demo)
{
    std::cout << "Run Levenberg-Marquardt solve y = exp(ax^2 + bx + c).\n";
    double a = 0.0, b = 0.0, c = 0.0; // 初值
    LevenbergMarquardt lm(a, b, c);
    LMCurveFitting demo;
    demo.RunDemo(lm);
}

} // namespace g2o
} // namespace xslam