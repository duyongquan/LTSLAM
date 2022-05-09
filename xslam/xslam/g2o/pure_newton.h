#ifndef XSLAM_G2O_PURE_NEWTON_H
#define XSLAM_G2O_PURE_NEWTON_H

#include <Eigen/Core>
#include <Eigen/Dense>

namespace xslam{
namespace g2o {

// https://github.com/TiantianUpup/numerical-optimization


// Function 
// f(x, y) =  x^2 + 3y^2 + 3xy + 4
class PureNewtonSolver
{
public:
  void RunDemo(bool visualization = false);


private:
    // 计算f(x, y)的函数值
    double ComputeFunctionValue(const Eigen::Vector2d& input);

    // Jacobian matrix
    Eigen::Vector2d ComputeJacobianMatrix(const Eigen::Vector2d& input);

    // 计算hessian矩阵
    // x: 迭代点
    Eigen::Matrix2d ComputeHessianMatrix(const Eigen::Vector2d& iter_x);

    Eigen::Vector2d state_;
};

} // namespace g2o
} // namespace xslam

#endif // XSLAM_G2O_PURE_NEWTON_H



