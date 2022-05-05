#ifndef XSLAM_G2O_PURE_NEWTON_H
#define XSLAM_G2O_PURE_NEWTON_H

#include "xslam/common/matplotlibcpp.h"

#include <Eigen/Core>
#include <Eigen/Dense>

namespace xslam{
namespace g2o {

// https://github.com/TiantianUpup/numerical-optimization


// Function 
// f(x, y) =  x^2 + 3y^2 + 3xy + 4
// f(x, y) = (x+1)^2 + (y+1)^2
// return np.sqrt(1+x[0]**2)+np.sqrt(1+x[1]**2)
class PureNewtonSolver
{
public:
  void RunDemo(bool visualization = false);

  // x0: 初始值
  bool NewtonMethod(const Eigen::Vector2d& x0);

private:
    // paint contour for function
    void DrawContour();
    
    // 计算f(x, y)的函数值
    double ComputeFunctionValue(const Eigen::Vector2d& input);

    // Jacobian matrix
    Eigen::Vector2d ComputeJacobianMatrix(const Eigen::Vector2d& input);

    // 计算hessian矩阵
    // input: 迭代点
    Eigen::Matrix2d ComputeHessianMatrix(const Eigen::Vector2d& input);

    Eigen::Vector2d state_;
};

} // namespace g2o
} // namespace xslam

#endif // XSLAM_G2O_PURE_NEWTON_H
